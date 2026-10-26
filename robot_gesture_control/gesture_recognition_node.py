#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Gesture Recognition Node for TurtleBot 4.
Uses MediaPipe Hands and OpenCV to detect 6 gestures:
1. WAVE: Hand waving horizontally
2. POINTING_UP: Index finger pointing up
3. VICTORY: Index and Middle fingers extended (V sign)
4. THUMBS_UP: Thumb pointing up, other fingers folded
5. OPEN_PALM: Open palm (all fingers extended, static)
6. CLOSED_FIST: All fingers folded into a fist

Publishes confirmed gesture to /detected_gesture (std_msgs/msg/String).
"""

import collections
import math
import time
from typing import List, Optional, Tuple

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GestureRecognitionNode(Node):
    """ROS 2 Node for camera-based hand gesture recognition."""

    def __init__(self):
        super().__init__('gesture_recognition_node')

        # Declare ROS 2 parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('min_detection_confidence', 0.70)
        self.declare_parameter('min_tracking_confidence', 0.60)
        self.declare_parameter('debounce_frames', 5)
        self.declare_parameter('show_window', True)
        self.declare_parameter('wave_window_seconds', 0.8)
        self.declare_parameter('wave_min_oscillations', 2)
        self.declare_parameter('wave_min_span', 0.07)
        self.declare_parameter('publish_rate', 20.0)

        # Retrieve parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.min_det_conf = self.get_parameter('min_detection_confidence').value
        self.min_track_conf = self.get_parameter('min_tracking_confidence').value
        self.debounce_frames = max(1, self.get_parameter('debounce_frames').value)
        self.show_window = self.get_parameter('show_window').value
        self.wave_window_sec = self.get_parameter('wave_window_seconds').value
        self.wave_min_oscillations = self.get_parameter('wave_min_oscillations').value
        self.wave_min_span = self.get_parameter('wave_min_span').value
        pub_rate = max(1.0, self.get_parameter('publish_rate').value)

        # Publisher
        self.gesture_publisher = self.create_publisher(String, '/detected_gesture', 10)

        # MediaPipe Hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=self.min_det_conf,
            min_tracking_confidence=self.min_track_conf,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Webcam initialization
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera with ID {self.camera_id}!')
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.get_logger().info(f'Camera {self.camera_id} opened successfully.')

        # Buffers for gesture debouncing and wave trajectory tracking
        self.gesture_buffer = collections.deque(maxlen=self.debounce_frames)
        self.wrist_history = collections.deque(maxlen=40)  # Stores (timestamp, x_norm)
        self.last_confirmed_gesture = 'NONE'
        self.last_confidence = 0.0

        # Subscribe to robot FSM state for live visual feedback
        self.robot_fsm_state = 'IDLE'
        self.fsm_sub = self.create_subscription(
            String, '/robot_fsm_state', self._fsm_callback, 10
        )

        # FPS calculation
        self.prev_time = time.time()
        self.fps = 0.0

        # Processing loop timer
        timer_period = 1.0 / pub_rate
        self.timer = self.create_timer(timer_period, self.process_frame)
        self.get_logger().info('GestureRecognitionNode initialized and running.')

    def _fsm_callback(self, msg: String):
        """Updates internal record of robot FSM state for overlay rendering."""
        self.robot_fsm_state = msg.data.strip().upper()

    @staticmethod
    def _calc_dist(p1, p2) -> float:
        """Euclidean distance between two landmarks (normalized coordinates)."""
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def _is_finger_extended(self, tip, pip, mcp, wrist) -> bool:
        """Robust check if index/middle/ring/pinky finger is extended."""
        dist_tip_wrist = self._calc_dist(tip, wrist)
        dist_pip_wrist = self._calc_dist(pip, wrist)
        # Finger is extended if tip is further from wrist than PIP and tip is above PIP
        return (dist_tip_wrist > dist_pip_wrist * 1.18) and (tip.y < pip.y)

    def _detect_wave(self, current_time: float, current_x: float) -> Tuple[bool, float]:
        """
        Detect horizontal hand waving motion using trajectory oscillation.
        Returns: (is_wave, wave_confidence)
        """
        self.wrist_history.append((current_time, current_x))

        # Filter points within the recent time window (extended to 1.0s)
        recent_pts = [
            (t, x) for (t, x) in self.wrist_history
            if (current_time - t) <= max(1.0, self.wave_window_sec)
        ]
        if len(recent_pts) < 6:
            return False, 0.0

        x_vals = [p[1] for p in recent_pts]
        min_x = min(x_vals)
        max_x = max(x_vals)
        span = max_x - min_x

        # More forgiving horizontal span threshold
        min_span = min(0.05, self.wave_min_span)
        if span < min_span:
            return False, 0.0

        # Count direction reversals in horizontal velocity
        diffs = [x_vals[i] - x_vals[i - 1] for i in range(1, len(x_vals))]
        filtered_diffs = [d for d in diffs if abs(d) > 0.004]

        if len(filtered_diffs) < 3:
            return False, 0.0

        reversals = 0
        last_sign = filtered_diffs[0] > 0
        for d in filtered_diffs[1:]:
            curr_sign = d > 0
            if curr_sign != last_sign:
                reversals += 1
                last_sign = curr_sign

        if reversals >= self.wave_min_oscillations:
            conf = min(0.99, max(0.70, span / 0.15))
            return True, conf

        return False, 0.0

    def classify_gesture(self, landmarks, current_time: float) -> Tuple[str, float]:
        """
        Classifies hand landmarks into one of 6 gestures.
        Returns: (gesture_name, confidence)
        """
        wrist = landmarks[0]
        thumb_cmc, thumb_mcp, thumb_ip, thumb_tip = landmarks[1], landmarks[2], landmarks[3], landmarks[4]
        index_mcp, index_pip, index_dip, index_tip = landmarks[5], landmarks[6], landmarks[7], landmarks[8]
        mid_mcp, mid_pip, mid_dip, mid_tip = landmarks[9], landmarks[10], landmarks[11], landmarks[12]
        ring_mcp, ring_pip, ring_dip, ring_tip = landmarks[13], landmarks[14], landmarks[15], landmarks[16]
        pinky_mcp, pinky_pip, pinky_dip, pinky_tip = landmarks[17], landmarks[18], landmarks[19], landmarks[20]

        # Finger extension states
        index_ext = self._is_finger_extended(index_tip, index_pip, index_mcp, wrist)
        mid_ext = self._is_finger_extended(mid_tip, mid_pip, mid_mcp, wrist)
        ring_ext = self._is_finger_extended(ring_tip, ring_pip, ring_mcp, wrist)
        pinky_ext = self._is_finger_extended(pinky_tip, pinky_pip, pinky_mcp, wrist)

        # Thumb extension (upward or outwards)
        dist_thumb_wrist = self._calc_dist(thumb_tip, wrist)
        dist_thumb_mcp = self._calc_dist(thumb_mcp, wrist)
        thumb_upward = (thumb_tip.y < thumb_ip.y - 0.03) and (thumb_tip.y < thumb_mcp.y - 0.04)
        thumb_ext = dist_thumb_wrist > dist_thumb_mcp * 1.2

        # Horizontal oscillation for Wave detection (using middle MCP or wrist X)
        is_waving, wave_conf = self._detect_wave(current_time, mid_mcp.x)

        # 1. Check for Wave vs Open Palm
        # When 4 or 5 fingers are fully extended
        if index_ext and mid_ext and ring_ext and pinky_ext:
            if is_waving:
                return 'WAVE', wave_conf
            return 'OPEN_PALM', 0.95

        # 2. Check for Pointing Up: Only Index extended, others folded
        if index_ext and (not mid_ext) and (not ring_ext) and (not pinky_ext):
            # Index finger must point vertically upwards
            if abs(index_tip.x - index_mcp.x) < abs(index_tip.y - index_mcp.y) * 0.75:
                # Thumb should not be pointing upward
                if not thumb_upward:
                    return 'POINTING_UP', 0.92

        # 3. Check for Victory: Index and Middle extended, Ring and Pinky folded
        if index_ext and mid_ext and (not ring_ext) and (not pinky_ext):
            dist_v = self._calc_dist(index_tip, mid_tip)
            if dist_v > 0.035 and (not thumb_upward):
                return 'VICTORY', 0.93

        # 4. Check for Thumbs Up: 4 fingers folded, thumb pointing straight up
        four_fingers_folded = (not index_ext) and (not mid_ext) and (not ring_ext) and (not pinky_ext)
        if four_fingers_folded:
            # Thumb tip is higher than all other finger knuckles and pointing up
            if thumb_upward and (thumb_tip.y < index_mcp.y - 0.03):
                return 'THUMBS_UP', 0.94

        # 5. Check for Closed Fist: All fingers folded, thumb tucked
        if four_fingers_folded:
            # Thumb is folded across palm / near index MCP
            if (not thumb_upward) and (thumb_tip.y >= index_mcp.y - 0.03):
                return 'CLOSED_FIST', 0.91

        return 'NONE', 0.50

    def process_frame(self):
        """Captures frame, performs inference, debounces, and publishes result."""
        if not self.cap.isOpened():
            self.publish_gesture('NONE', 0.0)
            return

        ret, frame = self.cap.read()
        curr_time = time.time()
        dt = curr_time - self.prev_time
        self.fps = 1.0 / dt if dt > 0 else 0.0
        self.prev_time = curr_time

        if not ret or frame is None:
            self.publish_gesture('NONE', 0.0)
            return

        # Flip horizontally for natural mirror interaction
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        results = self.hands.process(rgb_frame)
        rgb_frame.flags.writeable = True

        raw_gesture = 'NONE'
        raw_conf = 0.0
        hand_landmarks = None

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            raw_gesture, raw_conf = self.classify_gesture(hand_landmarks.landmark, curr_time)

        # Debounce filter: require majority or consistency over buffer
        self.gesture_buffer.append(raw_gesture)
        counts = collections.Counter(self.gesture_buffer)
        most_common_gesture, freq = counts.most_common(1)[0]

        if freq >= max(1, self.debounce_frames - 1):
            confirmed_gesture = most_common_gesture
            confidence = raw_conf if raw_gesture == confirmed_gesture else 0.85
        else:
            confirmed_gesture = self.last_confirmed_gesture
            confidence = self.last_confidence * 0.9

        self.last_confirmed_gesture = confirmed_gesture
        self.last_confidence = confidence

        # Publish confirmed gesture
        self.publish_gesture(confirmed_gesture, confidence)

        # Display window
        if self.show_window:
            self._render_overlay(frame, hand_landmarks, confirmed_gesture, confidence, w, h)

    def publish_gesture(self, gesture: str, confidence: float):
        """Publishes gesture string to ROS 2 topic."""
        if not rclpy.ok():
            return
        msg = String()
        msg.data = gesture
        try:
            self.gesture_publisher.publish(msg)
        except Exception:
            pass

    def _render_overlay(self, frame, landmarks, gesture: str, confidence: float, w: int, h: int):
        """Draws UI overlay, skeleton, and gesture info."""
        if landmarks:
            self.mp_draw.draw_landmarks(
                frame,
                landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style(),
            )

        # Color coding for gestures
        color_map = {
            'WAVE': (255, 165, 0),         # Orange
            'POINTING_UP': (255, 255, 0),   # Cyan
            'VICTORY': (0, 255, 255),       # Yellow
            'THUMBS_UP': (0, 255, 0),       # Green
            'OPEN_PALM': (0, 0, 255),       # Red (STOP)
            'CLOSED_FIST': (200, 0, 200),   # Magenta (CANCEL)
            'NONE': (128, 128, 128),        # Gray
        }
        color = color_map.get(gesture, (200, 200, 200))

        # Top status bar background
        cv2.rectangle(frame, (0, 0), (w, 85), (20, 20, 20), -1)
        cv2.line(frame, (0, 85), (w, 85), color, 3)

        # Gesture and confidence text
        label = f'Gesture: {gesture}'
        cv2.putText(frame, label, (15, 34), cv2.FONT_HERSHEY_DUPLEX, 0.9, color, 2, cv2.LINE_AA)

        info_text = f'Conf: {int(confidence * 100)}% | FPS: {self.fps:.1f}'
        cv2.putText(frame, info_text, (15, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)

        # Robot FSM State Display
        fsm = self.robot_fsm_state
        fsm_color_map = {
            'IDLE': (180, 180, 180),
            'SELECTED_FORWARD_BACKWARD': (0, 255, 255),  # Yellow
            'SELECTED_CIRCLE': (0, 255, 255),            # Yellow
            'EXECUTING_GREETING': (0, 255, 0),           # Green
            'EXECUTING_FORWARD_BACKWARD': (0, 255, 0),   # Green
            'EXECUTING_CIRCLE': (0, 255, 0),             # Green
            'EMERGENCY_STOP': (0, 0, 255),               # Red
        }
        fsm_color = fsm_color_map.get(fsm, (200, 200, 200))
        fsm_label = f'Robot: {fsm}'
        cv2.putText(frame, fsm_label, (w - 330, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.62, fsm_color, 2, cv2.LINE_AA)

        # Dynamic Instruction Banner based on Robot State
        if fsm == 'SELECTED_FORWARD_BACKWARD':
            prompt = '>> SHOW THUMBS UP TO RUN 4 CYCLES <<'
            cv2.putText(frame, prompt, (w - 380, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2, cv2.LINE_AA)
        elif fsm == 'SELECTED_CIRCLE':
            prompt = '>> SHOW THUMBS UP TO RUN CIRCLE <<'
            cv2.putText(frame, prompt, (w - 360, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2, cv2.LINE_AA)
        elif 'EXECUTING' in fsm:
            prompt = '>> IN MOTION (PALM: Stop | FIST: Cancel) <<'
            cv2.putText(frame, prompt, (w - 380, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2, cv2.LINE_AA)
        elif fsm == 'EMERGENCY_STOP':
            prompt = '>> STOPPED (Show CLOSED_FIST to Reset) <<'
            cv2.putText(frame, prompt, (w - 380, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            prompt = 'Point: Fwd-Bwd | V: Circle | Wave: Hello'
            cv2.putText(frame, prompt, (w - 360, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        # Bottom help banner
        help_bg_y = h - 35
        cv2.rectangle(frame, (0, help_bg_y), (w, h), (30, 30, 30), -1)
        help_text = 'WAVE: Hello | POINT: Fwd-Bwd | V: Circle | THUMB: Confirm | PALM: Stop | FIST: Reset'
        cv2.putText(frame, help_text, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 255, 180), 1, cv2.LINE_AA)

        cv2.imshow('TurtleBot 4 Gesture Recognition', frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # 'q' or ESC
            self.get_logger().info("User requested exit via GUI ('q').")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        """Cleanup resources on node destruction."""
        try:
            if hasattr(self, 'cap') and self.cap.isOpened():
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
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
