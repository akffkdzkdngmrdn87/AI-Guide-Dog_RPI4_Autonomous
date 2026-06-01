# 자율주행 모바일 로봇(AI Guide Dog) 종단간(End-to-End) 학습 아키텍처 및 엣지 배포 백서

**작성일자:** 2026년 5월 6일 (최종 갱신: 2026년 6월 1일)
**프로젝트 책임자:** J2W
**수석 시스템 엔지니어:** J2W

본 백서는 제한된 엣지(Edge) 컴퓨팅 환경(Raspberry Pi 4)에서 오퍼레이터의 수동 조작 주행 및 초기 자율주행 실증 데이터(총 누적 32GB 규모)를 모방 학습(Behavioral Cloning)하여, 동적 장애물을 스키드 턴(Skid-turn)으로 회피하는 자율주행 시스템의 구축 로직을 명시합니다. 특히, 글로벌 오픈소스 생태계인 Hugging Face LeRobot 등의 최신 대규모 AI 모델들이 엣지 환경에서 야기하는 연산 병목 현상을 비판적으로 분석하고, 이를 타개하기 위한 초경량 다중 센서 융합(Multi-modal Fusion) 신경망 설계 및 최적화 배포 파이프라인을 확립합니다.

---

## 제 1장: 오픈소스 로보틱스 생태계 리뷰 및 엣지 환경 적용 한계 (비판적 고찰)

본 연구는 시스템 설계 초기 단계에서 글로벌 최대의 오픈소스 로보틱스 플랫폼인 **Hugging Face의 LeRobot 깃허브(GitHub) 생태계**를 면밀히 교차 검증하였습니다. LeRobot이 제시하는 종단간 모방 학습(End-to-End Behavioral Cloning) 데이터 수집 파이프라인과 철학은 본 프로젝트의 근간에 핵심적인 영감을 제공하였습니다.

그러나 고성능 연산 장치(GPU)를 전제로 개발된 해당 저장소의 최신 SOTA(State-of-the-Art) 모델들을 Raspberry Pi 4 환경에 직접 적용할 때 발생하는 치명적인 공학적 한계를 다음과 같이 규명하였으며, 이에 따라 독자적인 초경량 아키텍처의 필요성을 도출하였습니다.

1. **디퓨전 폴리시 (Diffusion Policy)의 한계:**
   * **구조적 한계:** 고정밀 로봇 암(Manipulator) 제어를 위해 고안된 구조로, 점진적 노이즈 제거(Iterative Denoising)를 위해 신경망을 다수 반복 통과해야 합니다.
   * **결과:** Raspberry Pi CPU 환경에서 0.5~1.0초 이상의 추론 지연(Inference Latency)을 유발하며, 이는 모바일 로봇의 조향 제어 발산(Over-steering) 및 오실레이션(Oscillation, 팽이춤 현상)을 초래합니다.
2. **ACT (Action Chunking with Transformers)의 한계:**
   * **구조적 한계:** 대규모 어텐션(Attention) 연산을 동반하는 트랜스포머 아키텍처를 채택하고 있습니다.
   * **결과:** 하드웨어 가속기(NPU/GPU)가 부재한 소형 엣지 환경에서는 프레임 드롭(Frame Drop) 및 심각한 시스템 과부하를 유발합니다.
3. **전통적 NavStack/SLAM 기반 모듈형 제어의 한계:**
   * **구조적 한계:** 인지, 판단, 제어가 분리된 수학적 경로 계획에 전적으로 의존합니다.
   * **결과:** 복잡한 환경에서 오퍼레이터가 직관적으로 수행한 '장애물 회피 본능' 등 휴리스틱(Heuristics)한 모방 학습 데이터를 활용할 수 없어 유연성이 저하됩니다.
4. **대형 비전 트랜스포머 (Heavy ViT, e.g., Tesla FSD)의 한계:**
   * **구조적 한계:** 초당 수십 조 회의 연산(TOPS)이 가능한 전용 칩셋 환경을 요구합니다.
   * **결과:** 가용 RAM이 4GB에 불과한 엣지 디바이스 적재 시 즉각적인 메모리 초과(OOM, Out Of Memory) 현상을 유발합니다.

---

## 제 2장: 초경량 다중 감각 융합 신경망 설계 (MobileNetV3 + Spatial Attention)

상기한 연산 지연 및 메모리 한계를 극복하고 LeRobot의 모방 학습 철학을 엣지 환경에 이식하기 위해, 단일 패스(Single Pass) 기반의 초고속 추론이 가능한 독자적 하이브리드 신경망(GuideDogBrain)을 다음과 같이 설계하였습니다.

1. **시각 특징 추출 (Vision Encoder):** `MobileNetV3 (Small)`
   * 무거운 트랜스포머 연산을 배제하고, 깊이별 분리 합성곱(Depthwise Separable Convolution)을 통해 전방 카메라 이미지의 시각적 특징 맵(Feature Map)을 고속으로 추출합니다.
2. **공간 집중 메커니즘 (Spatial Attention Module):**
   * 합성곱(Conv2d) 및 시그모이드(Sigmoid) 기반의 어텐션 레이어를 추가하여, 이미지 내 주요 장애물 및 회피 공간의 픽셀에 네트워크의 가중치(Focus)를 집중시킵니다.
3. **이종 센서 데이터 융합 (Multimodal Feature Fusion):**
   * 360도 LiDAR 점군 데이터를 8구역의 최소 거리 배열로 압축하고, 초음파(3채널), IMU 각속도, 목표 상태 벡터를 포함한 13차원 벡터를 전결합층(Fully Connected Layer)을 통해 시각 텐서(576차원)와 융합(Concatenation)합니다.
4. **최종 출력층 (Output Layer):**
   * 융합된 텐서를 처리하여 로봇의 제어 속도(`linear.x`, `angular.z`)를 직접 회귀 추론(Regression)합니다.

---

## 제 3장: 목표 조건부 상태 기계 (Goal-Conditioned FSM) 및 라벨링 기준

로봇이 복잡한 주행 환경에서 '전진'과 '복귀'의 방향성을 상실하는 인과적 혼란(Causal Confusion)을 방지하기 위해, 오도메트리 누적 데이터를 기반으로 주행 상태(Goal-State) 지시자를 라벨링합니다.

### [데이터 세트 자동 라벨링 3단계 기준]
* **상태 0 (전진 탐색 모드):** * **조건:** 출발점($X_{odom} \approx 0$)에서 선속도(`linear.x`)가 양수로 인가되는 시점.
    * **행동 지표:** 전방 장애물을 회피하며 목표 지점(약 4.5m)을 향해 직진 위주의 주행을 수행합니다.
* **상태 1 (목표 도달 및 스키드 턴 모드):**
    * **조건:** $X_{odom}$이 목표 임계치에 도달하고, 선속도가 0에 수렴하며 IMU Z축 각속도 누적값이 약 $180^\circ (\pi)$에 도달하는 구간.
    * **행동 지표:** 목적지 도달을 인식하고 즉각적인 180도 제자리 회전(U-turn)을 수행합니다.
* **상태 2 (복귀 모드):**
    * **조건:** 회전 기동이 완료된 시점부터 기체의 좌표가 출발점($X_{odom} \approx 0$)에 수렴하여 제어 명령(`cmd_vel`)이 소거될 때까지의 구간.
    * **행동 지표:** 잔여 장애물을 회피하며 초기 출발 위치로 귀환합니다.

---

## 제 4장: 신경망 훈련 최적화 및 엣지 배포 파이프라인

수집된 대규모 주행 로그(`.db3`)를 기반으로 오프라인 워크스테이션에서 모델을 훈련하고, 엣지 디바이스로 이식하는 통합 파이프라인입니다.

### 1단계: 대규모 데이터 전처리 및 제련 (Data Preprocessing)
학습 서버(RTX 4070) 환경에서 **총 32GB 규모(약 150분 분량)**의 방대한 주행 데이터를 동기화합니다. 해당 데이터셋은 오퍼레이터의 수동 조작에 의한 **50회 왕복 주행 로그(1회 왕복 약 2분 소요, 총 100분)**와 **10회의 초기 자율주행 실증 로그(1회 약 5분 소요, 총 50분)**로 구성됩니다. 카메라 이미지(RGB)와 다중 센서 배열, 그리고 조향 제어값(Twist)을 정밀하게 매칭하고, 제 3장의 FSM 기준에 따라 목표 상태(Goal-State) 라벨을 주입한 최종 `master_dataset.csv`를 생성합니다.

### 2단계: 하이브리드 신경망 훈련 및 손실 함수 최적화 (Model Training)
PyTorch 프레임워크를 기반으로 모방 학습(Behavioral Cloning)을 수행합니다.
* **동적 가중치 페널티 (Reward-Weighted Loss):** 전방 0.2m 이내의 임박한 충돌 프레임에 대해서는 손실값(MSE)에 2.5배의 가중치를 부여하여 보수적인 회피 기동을 학습시키며, 목적지 전환 동작에는 보상(0.5배)을 인가합니다.
* **학습률 스케줄링 (Cosine Annealing Warm Restarts):** 모델이 지역 최적점(Local Minima)에 고착되는 것을 방지하기 위해 주기적으로 학습률을 초기화하며, 이는 검증 오차(Validation Loss) 그래프의 주기적 변동성(Spike)으로 나타나나 최종적인 일반화 성능(Accuracy)을 크게 향상시킵니다.

### 3단계: 가중치 압축 및 엣지 이식 (ONNX Export)
학습이 완료된 최적 가중치 파일(`.pth`)을 하드웨어 의존성이 낮고 추론 속도가 빠른 **ONNX(Open Neural Network Exchange)** 규격(`guide_dog_brain.onnx`)으로 변환(Export)하여 엣지 디바이스의 저장소로 이관합니다.

### 4단계: 실전 자율주행 추론 (Edge Inference)
Raspberry Pi 4 엣지 환경에서 PyTorch 라이브러리 로드를 배제하고, 경량화된 `onnxruntime` 엔진만을 활용하는 메인 자율주행 노드(`autonomous_drive.py`)를 가동합니다. 이를 통해 제한된 리소스 내에서도 10~15 FPS 수준의 실시간 인퍼런스(Inference)를 확보하며, 시스템 락(Stuck) 발생 시 VFH(Vector Field Histogram) 기반 강제 탈출 로직을 병행 가동하여 완벽한 무인 자율주행 임무를 수행합니다.

---

## 제 5장: 참고문헌 및 오픈소스 생태계 출처 (References & Acknowledgments)

본 자율주행 프로젝트는 전 세계 오픈소스 커뮤니티의 헌신적인 기여와 글로벌 연구 기관의 선행 학술 연구를 바탕으로 구현되었습니다. 특히 핵심 종단간(End-to-End) 자율주행 아키텍처 및 다중 센서 융합 시스템 수립에 결정적인 영감을 제공한 기술적 출처와 상세 코드 링크를 아래와 같이 명시합니다.

1. **Hugging Face LeRobot (모방 학습 및 데이터셋 파이프라인):**
   * **참조 영역:** 데이터셋 규격화 및 스트리밍 알고리즘(`src/lerobot/datasets/`) 및 종단간 모방 학습 정책 아키텍처(`src/lerobot/policies/`).
   * **상세 주소 (URL):**
     * 데이터셋 파이프라인 소스: [https://github.com/huggingface/lerobot/tree/main/src/lerobot/datasets]
     * 모방 학습(BC) 정책 소스: [https://github.com/huggingface/lerobot/tree/main/src/lerobot/policies]
   * **적용 내용:** 비전 텐서 및 이종 센서 데이터를 조향 제어값(Action)과 타임스탬프 기반으로 동기화하는 데이터 파이프라인의 구조적 영감을 제공받았습니다. 단, 엣지 환경의 한계를 고려하여 해당 저장소의 무거운 ACT 및 Diffusion Policy 모듈은 배제하고, 독자적인 경량화 모듈(MobileNetV3)로 백본을 완전히 대체하였습니다.

2. **Stanford University (Mobile ALOHA 프로젝트):**
   * **참조 영역:** Zipeng Fu et al. (2024)의 Bimanual Mobile Manipulation 텔레오퍼레이션 데이터 수집 및 로깅 스크립트(`aloha_scripts/`).
   * **상세 주소 (URL):** [https://github.com/MarkFzp/mobile-aloha/tree/main/aloha_scripts]
   * **적용 내용:** 오퍼레이터의 수동 개입(Human Demonstration) 데이터를 오차 없이 로깅(Logging)하는 시스템 아키텍처를 교차 검증하는 데 활용되었습니다. 수동 주행 데이터의 품질이 모방 학습 모델의 성능을 결정한다는 이론적 배경을 입증합니다.

3. **NVIDIA DAVE-2 (PilotNet - 종단간 자율주행 회귀 모델):**
   * **참조 영역:** Bojarski et al., "End to End Learning for Self-Driving Cars" (2016) 연구 논문 기반 오픈소스 구현체.
   * **상세 주소 (URL):** [https://github.com/tech-rules/DAVE2-Keras] (Keras/PyTorch 참조 구현체 예시)
   * **적용 내용:** 다차원 센서 퓨전 없이 전방 카메라 이미지로부터 직접 조향각 및 속도를 회귀(Regression) 추론하는 종단간(E2E) 아키텍처의 학술적 원형입니다. 본 프로젝트의 비전 특징 추출 층(Feature Extraction Layer)과 제어 속도 출력층 설계에 핵심적인 수학적 근거를 제공하였습니다.

4. **VFH (Vector Field Histogram) 국소 회피 알고리즘:**
   * **참조 영역:** J. Borenstein & Y. Koren (1991), University of Michigan 연구 논문 및 ROS 2 Nav2 스택의 DWB 로컬 플래너(Local Planner) 소스 코드.
   * **상세 주소 (URL):** [https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller]
   * **적용 내용:** 2D LiDAR 점군 데이터를 극좌표 히스토그램(Polar Histogram)으로 변환하여 개방된 회피 벡터를 산출하는 알고리즘으로, 본 프로젝트의 물리적 끼임(Stuck) 감지 및 지능형 제자리 회전 탈출 로직의 수학적 근간으로 적용되었습니다.

5. **ROS 2 Humble Hawksbill & ONNX Runtime:**
   * **설명:** 모바일 로봇의 이기종 분산 통신을 위한 표준 미들웨어 프레임워크(ROS 2) 및 Raspberry Pi 4 엣지 디바이스 환경에서의 초고속 경량 신경망 추론을 위한 최적화 엔진(ONNX Runtime).
   * **상세 주소 (URL):**
     * ROS 2 코어: [https://github.com/ros2]
     * ONNX Runtime: [https://github.com/microsoft/onnxruntime]

6. **프로젝트 데이터셋 글로벌 아카이빙 (Zenodo):**
   * **설명:** 본 프로젝트에서 자체 수집된 32GB 규모의 자율주행 실증 데이터셋 및 라이선스의 영구 보존을 위해 CERN(유럽입자물리연구소) 주도의 오픈 액세스 저장소에 등재를 완료하였습니다.
   * **상세 주소 (DOI URL):** [https://doi.org/10.5281/zenodo.20461774]
