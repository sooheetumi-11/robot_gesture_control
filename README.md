# robot_gesture_control

Package ROS 2 Python điều khiển robot **TurtleBot 4** (thực tế hoặc mô phỏng trên **Gazebo Fortress**) bằng cử chỉ bàn tay thông qua thị giác máy tính (**OpenCV** & **MediaPipe Hands**) kết hợp Máy trạng thái hữu hạn (**FSM - Finite State Machine**).

---

## 1. Cấu trúc Package

```text
robot_gesture_control/
├── README.md                           # Hướng dẫn cài đặt, build và vận hành
├── package.xml                         # Khai báo metadata và ROS 2 dependencies
├── setup.py                            # Cấu hình cài đặt và console scripts
├── setup.cfg                           # Chỉ định thư mục cài đặt thực thi
├── resource/
│   └── robot_gesture_control           # Resource marker
├── config/
│   └── gesture_params.yaml             # Cấu hình tham số camera, thuật toán và động học
├── launch/
│   └── gesture_control.launch.py       # Launch file khởi động đồng thời cả 2 node
└── robot_gesture_control/
    ├── __init__.py
    ├── gesture_recognition_node.py     # Node thị giác: nhận diện 6 cử chỉ & hiển thị HUD
    └── robot_behavior_node.py          # Node điều khiển: FSM, watchdog, an toàn & cmd_vel
```

---

## 2. Yêu cầu Hệ thống & Cài đặt Dependencies

* **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2:** ROS 2 Humble Hawksbill
* **Mô phỏng:** Gazebo Fortress (`ros_gz_bridge`)
* **Phần cứng:** Webcam (tích hợp hoặc USB camera hỗ trợ UVC tại `/dev/video0`)

### 2.1. Cài đặt thư viện Python (OpenCV & MediaPipe)
```bash
pip3 install opencv-python mediapipe
```

### 2.2. Kiểm tra và cấp quyền truy cập Webcam
Kiểm tra thiết bị camera trên máy tính:
```bash
ls -la /dev/video*
```
Nếu gặp lỗi `Permission denied` khi mở camera, thêm user hiện tại vào nhóm `video`:
```bash
sudo usermod -aG video $USER
newgrp video
```

### 2.3. Cài đặt các gói ROS 2 liên quan
```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-ros-gz-bridge
```

---

## 3. Hướng dẫn Biên dịch (Build)

Di chuyển vào thư mục workspace (ví dụ `~/workspaces/turtlebot4_ws` hoặc `/home/soohee/Turtlebot4`):

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash

# Build riêng package robot_gesture_control
colcon build --packages-select robot_gesture_control --symlink-install
```

Sau khi build thành công, thiết lập môi trường:
```bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

---

## 4. Hướng dẫn Khởi chạy Hệ thống

Khuyến nghị sử dụng **3 Terminal** độc lập. Trong mỗi terminal, luôn chạy khối lệnh chuẩn bị:
```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
```

### Bước 1: Khởi chạy Mô phỏng Gazebo (Terminal 1)
```bash
ros2 launch robot_sim sim.launch.py enable_camera:=true
```
*(Đợi Gazebo mở lên hoàn tất và TurtleBot 4 xuất hiện trên bản đồ).*

### Bước 2: Khởi chạy Hệ thống Điều khiển Cử chỉ (Terminal 2)
Chạy toàn bộ hệ thống bằng launch file:
```bash
ros2 launch robot_gesture_control gesture_control.launch.py
```
> Cửa sổ camera OpenCV sẽ xuất hiện hiển thị khung xương bàn tay, tên cử chỉ, độ tin cậy và thông báo hướng dẫn trạng thái robot bằng màu sắc.

#### Các tham số có thể tùy biến khi launch:
* `camera_id`: Chỉ số camera (mặc định `0` cho `/dev/video0`, đổi thành `1` nếu dùng webcam ngoài).
* `show_window`: Hiển thị cửa sổ OpenCV HUD (`true` hoặc `false`).
* `linear_speed`: Vận tốc dài robot (mặc định `0.22` m/s).
* `angular_speed`: Vận tốc góc robot (mặc định `0.45` rad/s).
* `watchdog_timeout`: Thời gian tự ngắt khi mất kết nối camera (mặc định `1.5` giây).

*Ví dụ dùng webcam ngoài cổng 1 và tăng tốc độ:*
```bash
ros2 launch robot_gesture_control gesture_control.launch.py camera_id:=1 linear_speed:=0.25
```

### Bước 3: Giám sát Topic thời gian thực (Terminal 3 - Tùy chọn)
```bash
# Xem cử chỉ tay nhận diện được:
ros2 topic echo /detected_gesture

# Xem trạng thái máy FSM của robot:
ros2 topic echo /robot_fsm_state

# Xem lệnh vận tốc gửi vào động cơ robot:
ros2 topic echo /cmd_vel
```

---

## 5. Bảng Ánh xạ Cử chỉ Tay & Hành vi Robot

Hệ thống hoạt động theo nguyên tắc an toàn **Chọn (Select) $\rightarrow$ Xác nhận (Confirm)** để ngăn robot tự ý chuyển động ngoài ý muốn:

| STT | Cử chỉ bàn tay | Tên cử chỉ (`/detected_gesture`) | Loại hành vi | Phản hồi của TurtleBot 4 trong Gazebo |
| :---: | :--- | :--- | :--- | :--- |
| **1** | **Vẫy bàn tay qua lại** | `WAVE` | Thực thi trực tiếp | Robot lập tức xoay đầu trái–phải ($2.4\,\text{s}$) để chào rồi tự động dừng về `IDLE`. |
| **2** | **Chỉ ngón trỏ lên** | `POINTING_UP` | Chọn hành vi (1) | **Robot đứng yên**. Cửa sổ HUD hiện nhắc màu vàng: `>> SHOW THUMBS UP TO RUN 4 CYCLES <<`. |
| **3** | **Giơ 2 ngón chữ V** | `VICTORY` | Chọn hành vi (2) | **Robot đứng yên**. Cửa sổ HUD hiện nhắc màu vàng: `>> SHOW THUMBS UP TO RUN CIRCLE <<`. |
| **4** | **Giơ ngón cái lên** | `THUMBS_UP` | Xác nhận chạy | • Nếu vừa chọn ngón trỏ: Robot tiến $40\,\text{cm}$ $\rightarrow$ lùi $40\,\text{cm}$ (lặp lại đúng 4 lần rồi dừng).<br>• Nếu vừa chọn chữ V: Robot chạy đúng 1 vòng tròn khép kín ($360^\circ$) rồi dừng. |
| **5** | **Xòe bàn tay đứng yên** | `OPEN_PALM` | Phanh khẩn cấp | **Robot dừng khựng lại ngay lập tức** trong mọi tình huống. |
| **6** | **Nắm chặt bàn tay** | `CLOSED_FIST` | Hủy lệnh & Reset | Hủy trạng thái đang chọn hoặc đang chạy dở, đưa robot về trạng thái chờ `IDLE`. |

---

## 6. Các Cơ chế An toàn Tích hợp

1. **Bộ lọc thời gian ân hạn (Grace Period):**
   * Sau khi chọn (`POINTING_UP` hoặc `VICTORY`), hệ thống dành ra $1.2\,\text{giây}$ ân hạn để người dùng thoải mái đổi dáng tay sang `THUMBS_UP` mà không sợ việc co/xòe ngón tay ngẫu nhiên làm hủy lệnh.
   * Sau khi bắt đầu chạy, hệ thống khóa nhận diện lệnh dừng trong $1.5\,\text{giây}$ đầu để người dùng hạ tay xuống bàn nghỉ ngơi mà không làm dừng robot.
2. **Bộ đệm làm mịn (Debounce Filter):**
   * Giữ hàng đợi 5 khung hình liên tiếp để lọc bỏ hiện tượng chập chờn (flickering).
3. **Bộ giám sát mất tín hiệu (Watchdog Safety Timer):**
   * Nếu luồng webcam bị ngắt quãng hoặc node nhận diện bị dừng quá $1.5\,\text{giây}$, node hành vi tự động phanh xe về $\vec{v} = 0, \vec{\omega} = 0$.
4. **Giới hạn vận tốc an toàn (Velocity Clamping):**
   * Giới hạn $v \le 0.30\,\text{m/s}$, $\omega \le 0.90\,\text{rad/s}$, chống lật xe trong mô phỏng Gazebo.
5. **Dừng an toàn khi thoát (Graceful Shutdown):**
   * Nhấn phím `q` trên cửa sổ OpenCV hoặc `Ctrl + C` tại terminal để thoát sạch sẽ, node tự động phát 5 lần gói tin dừng trước khi đóng tiến trình.

---

## 7. Xử lý Sự cố Thường gặp (Troubleshooting)

| Sự cố | Nguyên nhân | Cách khắc phục |
| :--- | :--- | :--- |
| `Cannot open camera with ID 0` | Webcam bận hoặc cổng khác | Đóng các ứng dụng đang chiếm webcam (Cheese, trình duyệt, Zoom). Thử đổi sang `camera_id:=1`. |
| Robot trong Gazebo không nhúc nhích | Khác biệt `ROS_DOMAIN_ID` | Kiểm tra tất cả các terminal đều đã chạy `export ROS_DOMAIN_ID=69`. |
| Cửa sổ OpenCV không hiển thị | Lỗi biến môi trường `DISPLAY` | Chạy lệnh `echo $DISPLAY` (kết quả chuẩn là `:0` hoặc `:1`). Nếu dùng SSH cần thêm cờ `ssh -X`. |
| Robot tiến lùi quá ngắn khó thấy | Dùng thông số cũ | Chạy `colcon build --packages-select robot_gesture_control` để cập nhật profile $40\,\text{cm}$. |
