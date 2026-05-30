# Autonomous Guide Dog Robot
**ROS 2 Multi-modal Sensor Fusion & End-to-End Autonomous Driving on Edge Device**

[![DOI](https://zenodo.org/badge/1254341248.svg)](https://doi.org/10.5281/zenodo.20461774)

## 🎬 자율주행 실증 데모 (Autonomous Driving in Action)
<video src="https://github.com/akffkdzkdngmrdn87/AI-Guide-Dog_RPI4_Autonomous/releases/download/v1.0.0/2026-05-29_자율주행_개선.mp4" controls="controls" width="100%" muted="muted"></video>

## 1. 프로젝트 일러두기 (Project Notice)

* **개발 목적:** 본 프로젝트는 상용화 목적이 아닌, 1인 연구 주도의 엣지(Edge) 인공지능 기반 자율주행 및 다중 센서 융합 기술의 실증(PoC, Proof of Concept)을 목적으로 진행되었습니다.
* **연구 범위:** 제한된 컴퓨팅 자원(Raspberry Pi 4, 4GB RAM) 환경에서 무거운 SLAM 기반 내비게이션에 의존하지 않고, 초경량 딥러닝 모델이 지연 없이(Zero-Latency) 실시간 동적 장애물 회피를 수행할 수 있음을 입증하는 데 초점을 맞추었습니다.
* **AI 지원 개발 방법론 (AI-Assisted Pair Programming):** 본 시스템 설계 및 소스 코드 작성의 90% 이상은 대형 언어 모델(Google Gemini Pro)과의 페어 프로그래밍을 통해 진행되었습니다. 입문 단기간 내에 이기종 통신망(ROS 2 - MCU)과 AI 추론 모델을 결합한 풀스택(Full-stack) 시스템 구축의 실효성을 입증하였습니다.
* **데이터셋 및 라이선스 보존:** 본 프로젝트는 Apache 2.0 라이선스를 따르며, Zenodo(CERN)의 글로벌 오픈 액세스 저장소에 공식 등재되어 영구적인 디지털 객체 식별자(DOI)를 부여받았습니다. 해당 라이선스 규정 범위 내에서 누구나 자유로운 활용 및 변형이 가능합니다.

## 2. 시스템 개발 환경 (System Environment)

크로스 컴파일 및 시스템 호환성을 최대한 확보하기 위해 호스트 PC와 엣지 디바이스의 OS 버전을 통일하였습니다.
* **호스트 PC (AI 학습 및 관제):** Ubuntu Linux 22.04.5 LTS (64-bit Desktop)
* **엣지 디바이스 (Raspberry Pi 4):** Ubuntu Linux 22.04.5 LTS Server (64-bit, aarch64)
* **미들웨어 및 프레임워크:** ROS 2 Humble, ONNX Runtime, PyTorch

## 3. 핵심 아키텍처 및 알고리즘 (Core Architecture)

1. **초경량 VLA 융합 모델 (MobileNetV3 + LSTM):** 비전 트랜스포머(ViT) 등 무거운 아키텍처를 배제하고, 단일 스캔으로 특징을 추출하는 CNN과 과거 궤적을 기억하는 LSTM을 결합하였습니다. ONNX Runtime 1-Thread 할당을 통해 엣지 디바이스에서 12.5 FPS 이상의 안정적인 추론 속도를 확보했습니다.
2. **지능형 국소 최적점 탈출 (VFH Anti-Stuck Logic):** 물리적 끼임(Stuck) 발생 시, 라이다(LiDAR) 8구역 스캔 데이터 중 가장 넓은 여유 공간을 탐색하여 제자리 스키드 턴(Skid-turn)으로 회피하는 페일세이프(Failsafe) 알고리즘을 구현하였습니다.
3. **가상 안전 터널 (Virtual Safety Tube):** 기구학적 제원(전폭 21.6cm)을 바탕으로 수학적 데이터 필터링(`|y| = |r · sin(θ)| < 0.128m`)을 적용, 측면 벽면을 전방 장애물로 오인하는 현상을 방지하였습니다.

## 4. 퀵 스타트 (Quick Start)

시스템을 구동하기 전, 반드시 `필독.md`를 참고하여 ZRAM 및 커널 파라미터 설정을 완료해야 합니다.

```bash
# 1. 하드웨어 I/O 권한 부여
sudo chmod 777 /dev/ttyUSB* /dev/ttyACM* /dev/video0 /dev/gpiomem

# 2. 센서 및 통신망 가동 (별도의 터미널 창에서 각각 실행)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.15 --roll 0 --pitch 0 --yaw 1.5708 --frame-id base_link --child-frame-id laser
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_baudrate:=460800
python3 src/real_odom_publisher.py
python3 src/motor_bridge.py

# 3. AI 자율주행 메인 노드 가동
python3 src/autonomous_drive.py
