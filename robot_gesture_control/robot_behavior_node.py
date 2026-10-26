#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Robot Behavior Node for TurtleBot 4.
Subscribes to /detected_gesture (std_msgs/msg/String) and controls
the robot via /cmd_vel (geometry_msgs/msg/Twist) using a Finite State Machine (FSM).

Safety features:
- Finite State Machine with confirmation logic
- Edge-triggered gesture transitions to prevent repeated triggers
- Watchdog timer to automatically stop robot on signal/camera loss
- Bounded velocity limits
- Deterministic finite termination conditions for all behaviors
- Emergency stop on Open Palm and clean cancellation on Closed Fist
"""

import math
import time
from enum import Enum
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotState(Enum):
    IDLE = 'IDLE'
    SELECTED_FORWARD_BACKWARD = 'SELECTED_FORWARD_BACKWARD'
    SELECTED_CIRCLE = 'SELECTED_CIRCLE'
    EXECUTING_GREETING = 'EXECUTING_GREETING'
    EXECUTING_FORWARD_BACKWARD = 'EXECUTING_FORWARD_BACKWARD'
    EXECUTING_CIRCLE = 'EXECUTING_CIRCLE'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


class RobotBehaviorNode(Node):
    """FSM-based behavioral controller for TurtleBot 4."""

    def __init__(self):
        super().__init__('robot_behavior_node')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gesture_topic', '/detected_gesture')
        self.declare_parameter('linear_speed', 0.15)          # m/s
        self.declare_parameter('angular_speed', 0.40)         # rad/s
        self.declare_parameter('max_linear_speed', 0.30)      # Safety clamp
        self.declare_parameter('max_angular_speed', 0.90)     # Safety clamp
        self.declare_parameter('watchdog_timeout', 1.5)       # seconds
        self.declare_parameter('forward_backward_cycles', 4)  # 4 repetitions
        self.declare_parameter('step_duration', 1.8)          # seconds per forward/back step (approx 40cm)
        self.declare_parameter('pause_duration', 0.3)         # seconds pause between steps
        self.declare_parameter('control_rate', 20.0)          # Hz

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        gesture_topic = self.get_parameter('gesture_topic').value
        self.linear_speed = min(
            self.get_parameter('linear_speed').value,
            self.get_parameter('max_linear_speed').value,
        )
        self.angular_speed = min(
            self.get_parameter('angular_speed').value,
            self.get_parameter('max_angular_speed').value,
        )
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.fb_cycles = self.get_parameter('forward_backward_cycles').value
        self.step_duration = self.get_parameter('step_duration').value
        self.pause_duration = self.get_parameter('pause_duration').value
        control_rate = max(5.0, self.get_parameter('control_rate').value)

        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '/robot_fsm_state', 10)
        self.gesture_sub = self.create_subscription(
            String, gesture_topic, self.gesture_callback, 10
        )

        # State machine variables
        self.state = RobotState.IDLE
        self.last_gesture: Optional[str] = None
        self.last_msg_time = time.time()
        self.is_watchdog_triggered = False

        # Execution tracking variables
        self.action_start_time = 0.0
        self.selection_time = 0.0
        self.current_cycle = 0
        self.sub_step = 0  # 0: forward, 1: pause, 2: backward, 3: pause
        self.sub_step_start_time = 0.0

        # Duration parameters for behaviors
        self.greeting_duration = 2.4
        self.circle_duration = (2.0 * math.pi) / max(0.01, self.angular_speed)

        # Periodic control loop
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        self.get_logger().info(
            f'RobotBehaviorNode started. Listening on {gesture_topic}, publishing to {cmd_vel_topic}.'
        )
        self.get_logger().info('Initial State: IDLE. Ready for gestures.')

    def gesture_callback(self, msg: String):
        """Processes incoming gesture events."""
        current_time = time.time()
        self.last_msg_time = current_time

        if self.is_watchdog_triggered:
            self.get_logger().info('Watchdog recovered: Gesture signal resumed.')
            self.is_watchdog_triggered = False

        gesture = msg.data.strip().upper()

        # Edge-triggered detection: ignore if identical to previous raw input
        if gesture == self.last_gesture:
            return

        self.last_gesture = gesture
        self.get_logger().info(f'Received Gesture: [{gesture}] in State: [{self.state.value}]')
        self.handle_fsm_transition(gesture)

    def handle_fsm_transition(self, gesture: str):
        """Finite State Machine transition logic with transition grace periods."""
        now = time.time()

        # Check if robot is currently executing an action
        is_executing = self.state in (
            RobotState.EXECUTING_GREETING,
            RobotState.EXECUTING_FORWARD_BACKWARD,
            RobotState.EXECUTING_CIRCLE,
        )
        # 1.5s grace period immediately after start: ignore brief hand-lowering gestures
        exec_in_grace = is_executing and ((now - self.action_start_time) < 1.5)

        # Check if robot is in selected state
        is_selected = self.state in (
            RobotState.SELECTED_FORWARD_BACKWARD,
            RobotState.SELECTED_CIRCLE,
        )
        # 1.2s grace period after selecting: don't accidentally cancel while transitioning fingers
        select_in_grace = is_selected and ((now - self.selection_time) < 1.2)

        # --- GLOBAL COMMANDS ---

        # 1. OPEN_PALM: Emergency Stop
        if gesture == 'OPEN_PALM':
            if exec_in_grace or select_in_grace:
                self.get_logger().debug('OPEN_PALM ignored during start/selection grace period.')
                return
            self.get_logger().warn('EMERGENCY STOP requested via OPEN_PALM gesture!')
            self.transition_to(RobotState.EMERGENCY_STOP)
            self.stop_robot()
            return

        # 2. CLOSED_FIST: Cancel current behavior and reset to IDLE
        if gesture == 'CLOSED_FIST':
            if exec_in_grace or select_in_grace:
                self.get_logger().debug('CLOSED_FIST ignored during start/selection grace period.')
                return
            self.get_logger().info('CANCELLATION requested via CLOSED_FIST gesture. Resetting to IDLE.')
            self.transition_to(RobotState.IDLE)
            self.stop_robot()
            return

        # --- STATE-DEPENDENT TRANSITIONS ---

        if self.state in (RobotState.IDLE, RobotState.EMERGENCY_STOP):
            if gesture == 'WAVE':
                self.get_logger().info('Wave detected: Starting greeting rotation (left-right).')
                self.action_start_time = time.time()
                self.transition_to(RobotState.EXECUTING_GREETING)

            elif gesture == 'POINTING_UP':
                self.get_logger().info('Selected: Forward-Backward (4 cycles). Show THUMBS_UP to confirm.')
                self.selection_time = time.time()
                self.transition_to(RobotState.SELECTED_FORWARD_BACKWARD)

            elif gesture == 'VICTORY':
                self.get_logger().info('Selected: Circular Trajectory. Show THUMBS_UP to confirm.')
                self.selection_time = time.time()
                self.transition_to(RobotState.SELECTED_CIRCLE)

            elif gesture == 'THUMBS_UP':
                self.get_logger().warn('THUMBS_UP ignored: No action currently selected.')

        elif self.state == RobotState.SELECTED_FORWARD_BACKWARD:
            if gesture == 'THUMBS_UP':
                self.get_logger().info('CONFIRMED! Executing Forward-Backward (4 cycles)...')
                self.current_cycle = 0
                self.sub_step = 0
                self.sub_step_start_time = time.time()
                self.action_start_time = time.time()
                self.transition_to(RobotState.EXECUTING_FORWARD_BACKWARD)

            elif gesture == 'VICTORY':
                self.get_logger().info('Switching selection: Circular Trajectory. Show THUMBS_UP to confirm.')
                self.selection_time = time.time()
                self.transition_to(RobotState.SELECTED_CIRCLE)

        elif self.state == RobotState.SELECTED_CIRCLE:
            if gesture == 'THUMBS_UP':
                self.get_logger().info('CONFIRMED! Executing Circular Trajectory (1 full revolution)...')
                self.action_start_time = time.time()
                self.transition_to(RobotState.EXECUTING_CIRCLE)

            elif gesture == 'POINTING_UP':
                self.get_logger().info('Switching selection: Forward-Backward. Show THUMBS_UP to confirm.')
                self.selection_time = time.time()
                self.transition_to(RobotState.SELECTED_FORWARD_BACKWARD)

        elif self.state in (
            RobotState.EXECUTING_GREETING,
            RobotState.EXECUTING_FORWARD_BACKWARD,
            RobotState.EXECUTING_CIRCLE,
        ):
            # While actively executing an action, selection gestures are ignored
            if gesture in ('POINTING_UP', 'VICTORY', 'THUMBS_UP', 'WAVE'):
                self.get_logger().debug(f'Ignored [{gesture}]: Currently executing [{self.state.value}].')

    def transition_to(self, new_state: RobotState):
        """Updates FSM state with logging and publishes to /robot_fsm_state."""
        self.get_logger().info(f'FSM State: {self.state.value} -> {new_state.value}')
        self.state = new_state
        if rclpy.ok():
            msg = String()
            msg.data = new_state.value
            try:
                self.state_pub.publish(msg)
            except Exception:
                pass

    def control_loop(self):
        """Periodic control loop at specified rate."""
        try:
            now = time.time()

            # Watchdog check: ensure camera / gesture node is still sending data
            if (now - self.last_msg_time) > self.watchdog_timeout:
                if not self.is_watchdog_triggered:
                    self.get_logger().warn(
                        f'WATCHDOG TIMEOUT ({self.watchdog_timeout}s elapsed without gesture signal). Stopping robot!'
                    )
                    self.is_watchdog_triggered = True
                    self.stop_robot()
                return

            # Execute behavioral logic based on current state
            if self.state in (RobotState.IDLE, RobotState.SELECTED_FORWARD_BACKWARD, RobotState.SELECTED_CIRCLE, RobotState.EMERGENCY_STOP):
                self.stop_robot()

            elif self.state == RobotState.EXECUTING_GREETING:
                self._execute_greeting(now)

            elif self.state == RobotState.EXECUTING_FORWARD_BACKWARD:
                self._execute_forward_backward(now)

            elif self.state == RobotState.EXECUTING_CIRCLE:
                self._execute_circle(now)
        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {e}')
            self.stop_robot()

    def _execute_greeting(self, now: float):
        """Waving greeting: turns left, right, then returns to center with clear movement."""
        elapsed = now - self.action_start_time
        twist = Twist()

        greet_speed = max(0.55, self.angular_speed * 1.4)
        if elapsed < 0.6:
            # Turn left
            twist.angular.z = greet_speed
        elif elapsed < 1.8:
            # Turn right
            twist.angular.z = -greet_speed
        elif elapsed < 2.4:
            # Turn back left to center
            twist.angular.z = greet_speed
        else:
            # Done!
            self.stop_robot()
            self.get_logger().info('Greeting behavior finished. Returning to IDLE.')
            self.transition_to(RobotState.IDLE)
            return

        self.cmd_vel_pub.publish(twist)

    def _execute_forward_backward(self, now: float):
        """Executes forward-backward movement for exactly 4 cycles safely."""
        sub_elapsed = now - self.sub_step_start_time
        twist = Twist()

        # Sub-steps:
        # 0: Forward (linear_speed)
        # 1: Pause (0 m/s)
        # 2: Backward (-linear_speed)
        # 3: Pause (0 m/s)
        if self.sub_step == 0:
            if sub_elapsed < self.step_duration:
                twist.linear.x = self.linear_speed
            else:
                self.sub_step = 1
                self.sub_step_start_time = now
                self.stop_robot()
                return

        elif self.sub_step == 1:
            if sub_elapsed < self.pause_duration:
                twist.linear.x = 0.0
            else:
                self.sub_step = 2
                self.sub_step_start_time = now
                return

        elif self.sub_step == 2:
            if sub_elapsed < self.step_duration:
                twist.linear.x = -self.linear_speed
            else:
                self.sub_step = 3
                self.sub_step_start_time = now
                self.stop_robot()
                return

        elif self.sub_step == 3:
            if sub_elapsed < self.pause_duration:
                twist.linear.x = 0.0
            else:
                self.current_cycle += 1
                self.get_logger().info(
                    f'Completed cycle {self.current_cycle} / {self.fb_cycles}'
                )
                if self.current_cycle >= self.fb_cycles:
                    self.stop_robot()
                    self.get_logger().info(
                        f'Completed all {self.fb_cycles} forward-backward cycles. Returning to IDLE.'
                    )
                    self.transition_to(RobotState.IDLE)
                    return
                else:
                    self.sub_step = 0
                    self.sub_step_start_time = now
                    return

        self.cmd_vel_pub.publish(twist)

    def _execute_circle(self, now: float):
        """Moves robot in a circular arc for 1 complete revolution."""
        elapsed = now - self.action_start_time
        duration = getattr(self, 'circle_duration', (2.0 * math.pi) / max(0.01, self.angular_speed))
        if elapsed < duration:
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
        else:
            self.stop_robot()
            self.get_logger().info(
                f'Circular trajectory complete ({duration:.1f}s). Returning to IDLE.'
            )
            self.transition_to(RobotState.IDLE)

    def stop_robot(self):
        """Publishes zero velocities to safely halt the robot."""
        if not rclpy.ok():
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        try:
            self.cmd_vel_pub.publish(twist)
        except Exception:
            pass

    def destroy_node(self):
        """Safely stops robot when shutting down."""
        try:
            if rclpy.ok():
                self.get_logger().info('Shutting down: Sending stop commands to robot.')
                self.stop_robot()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
