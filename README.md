```markdown
# ArduCAM USB ROS2 Stereo Publisher

ArduCAM USB Shield 및 카메라 모듈(예: IMX708, AR1820 등)을 **ROS2 Humble/Foxy** 환경에서 사용하기 위한 스테레오 드라이버 패키지입니다. 본 패키지는 두 대의 카메라 영상을 동시에 캡처하여 `sensor_msgs/Image` 및 `sensor_msgs/CameraInfo` 메시지로 발행합니다.

## 📌 주요 특징
* **스테레오 지원:** 두 대의 카메라를 독립적인 노드 루프 내에서 동기화하여 발행합니다.
* **내부 파라미터 지원:** `.npy` 파일을 통해 카메라 캘리브레이션 정보(`CameraInfo`)를 함께 제공합니다.
* **유연한 설정:** 실행 시 해상도, 프레임 레이트, 설정 파일(.cfg)을 인자로 전달할 수 있습니다.

---

## 🛠 Prerequisites

패키지를 빌드하고 실행하기 위해 다음 라이브러리들이 필요합니다.

### 1. ArduCAM SDK & Config Parser
ArduCAM에서 제공하는 파이썬 설정 파서를 먼저 설치해야 합니다.
```bash
pip3 install arducam-config-parser numpy opencv-python

2. ROS2 Dependencies
이미지 변환 및 전송을 위한 표준 패키지들이 필요합니다.

sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

🏗 How to Build
작업 공간(Workspace)의 src 폴더에 클론한 뒤 빌드합니다.

cd ~/ros2_ws/src
git clone [https://github.com/ggun99/arducam_usb_ros2.git](https://github.com/ggun99/arducam_usb_ros2.git)

cd ~/ros2_ws
colcon build --symlink-install --packages-select arducam_usb_ros2
source install/setup.bash

🚀 How to Run
기본 제공되는 .cfg 파일과 함께 노드를 실행합니다. (파일명은 본인의 환경에 맞춰 변경하세요.)

ros2 run arducam_usb_ros2 arducam_node --config-file AR1820_MIPI_4Lane_RAW10_8b_1920x1080_120fps.cfg

⚙️ Parameters
실행 시 다음과 같은 인자를 추가하여 설정을 변경할 수 있습니다.

Parameter,Type,Default,Description
"-f, --config-file",string,(Required),ArduCAM 설정 파일 (.cfg)의 경로 또는 파일명
--rate,float,30.0,토픽 발행 주기 (Hz)
--left-serial,string,0002,왼쪽 카메라 시리얼 번호 (토픽명 및 .npy 매칭용)
--right-serial,string,0019,오른쪽 카메라 시리얼 번호
--flip,bool,False,이미지 좌우 반전 여부


📂 File Structure & Calibration
본 패키지는 실행 시 share/arducam_usb_ros2/config/ 디렉토리에서 .npy 파일을 자동으로 검색합니다.

Intrinsic Matrix: ac_sn_[SERIAL]_camera_mat.npy

Distortion Coefficients: ac_sn_[SERIAL]_dist_coefs.npy

캘리브레이션 파일이 없을 경우 기본값이 적용되거나 경고가 발생할 수 있으므로, 본인의 시리얼 번호에 맞는 파일을 arducam_usb_ros2/ 폴더 내에 배치한 뒤 빌드하세요.

📄 License
This project is licensed under the Apache License 2.0.

👤 Contact
Maintainer: Geon (xkrjs99@naver.com)

GitHub: ggun99
