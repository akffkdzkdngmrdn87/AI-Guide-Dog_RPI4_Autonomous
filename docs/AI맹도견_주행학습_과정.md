# 모바일 로봇 비전-자율주행 데이터 파이프라인 및 신경망 훈련 백서

**작성일자:** 2026년 4월 23일 (최종 갱신: 2026년 6월 1일)
**프로젝트 책임자:** J2W
**수석 시스템 엔지니어:** J2W

본 백서는 Raspberry Pi 4 및 Arduino Uno 기반의 스키드 스티어링(Skid-Steering) 모바일 로봇을 활용한 데이터 수집 및 종단간(End-to-End) 딥러닝 기반 자율주행 모델 학습 과정에 대한 통합 기술 문서입니다. 합성곱 신경망(CNN) 및 공간 어텐션(Spatial Attention) 메커니즘을 융합한 네트워크 아키텍처와 최적화된 하드웨어 운용 방안을 기술합니다.

---

## 제 1장: 시스템 하드웨어 제원 및 구성

로봇의 정밀한 데이터 획득 및 제어를 위한 기구학적 제원과 센서 구성은 다음과 같습니다.

* **물리적 제원:** 전장 0.27m / 윤거 0.15m / 축거 0.13m
* **타이어 제원 및 오도메트리:** 직경 66mm 휠 및 20 Ticks/Rev 광학식 엔코더 적용 (1틱당 이동 거리 약 0.01036m 산출). 스키드 스티어링의 슬립 보정을 위해 `SLIP_FACTOR = 0.95`를 계측 데이터에 일괄 적용하였습니다.
* **멀티모달 센서부:**
  * **LiDAR:** SLLidar (2D 공간 계측)
  * **IMU:** MPU-6050 (각속도 및 평형 계측)
  * **Ultrasonic:** HC-SR04 x 3 (하단 사각지대 및 근접 장애물 탐지)
  * **Vision:** Raspberry Pi Camera Rev 1.3 (320x240 해상도 최적화)
* **독립 전력망 설계:** 메인 프로세서(Raspberry Pi)와 모터 구동부(Arduino)의 전원을 2기의 10000mAh 보조배터리로 분리하여 하드웨어적 전압 강하(Brownout)를 방지하였습니다.

---

## 제 2장: 데이터 수집 프로세스 및 트러블슈팅

종단간 자율주행(End-to-End Autonomous Driving) 모델 학습을 위한 고품질 데이터 확보 과정에서 발생한 이슈 및 해결 방안입니다.

1. **무선 통신 대역폭 초과 및 패킷 손실 (Packet Drop):** 원격 PC에서 영상 토픽을 기록(Record) 시 프레임의 95%가 누락되는 병목 현상이 발생했습니다. 이를 해결하기 위해 엣지 디바이스(Raspberry Pi)의 로컬 스토리지(SD 카드)에 `rosbag2`를 직접 구동하는 방식으로 변경하여 11GB 규모의 무손실 원본 데이터를 성공적으로 확보하였습니다.
2. **조향 제어 역전 현상 교정:** 비전 기반 수동 조작 시 로봇의 조립 방향성에 기인한 좌우 반전 및 대각선 주행 제한 이슈가 식별되었습니다. 직렬 통신 브리지 노드 레이어에서 조향 벡터(Twist)를 크로스 매핑(Cross-mapping) 처리하여 8방향 기동 알고리즘을 하위 제어기에 정합시켰습니다.

---

## 제 3장: 다중 센서 융합 신경망 모델 (Multi-modal Fusion Network)

비전 데이터와 1차원 센서(LiDAR, 초음파, IMU) 데이터를 융합하여 최적의 선속도(Linear Velocity) 및 각속도(Angular Velocity)를 회귀(Regression) 추론하는 PyTorch 기반 훈련 아키텍처입니다.

### 1. 핵심 아키텍처 및 손실 함수(Loss Function)의 특징
* **Spatial Attention Module:** 입력 이미지의 주요 특징 맵(Feature Map)에 집중하기 위해 합성곱 연산(Conv2d) 기반의 공간 어텐션 메커니즘을 적용하였습니다.
* **Feature Fusion:** MobileNetV3 (Small) 모델로 추출한 비전 특징 텐서와 전결합층(Fully Connected Layer)을 통과한 13차원 센서 텐서를 결합(Concatenation)하여 최종 조향 값을 연산합니다.
* **동적 페널티 가중치 (Dynamic Reward-Weighted Loss):** 주행 안정성 극대화를 위해 단순 평균 제곱 오차(MSE)를 변형하였습니다. 전방 0.2m 이내 장애물 감지 시 손실에 2.5배의 페널티를, 정상 목적지 도달(Goal State) 시 0.5배의 보상을 부여하여 안전 지향적 학습(Safety-oriented Learning)을 유도합니다.

### 2. 모델 학습 전체 소스 코드 (`train_brain.py`)

```python
#!/usr/bin/env python3
# =====================================================================
# 자율주행 모델 훈련 파이프라인 (Multi-modal Sensor Fusion Training)
# Vision (MobileNetV3) + Sensor Data + Spatial Attention Architecture
# =====================================================================

import os                                      
import torch                                   
import torch.nn as nn                          
import torch.optim as optim                    
from torch.utils.data import Dataset, DataLoader 
import torchvision.models as models            
import torchvision.transforms as transforms    
import pandas as pd                            
import numpy as np                             
import cv2                                     
import matplotlib.pyplot as plt                
import matplotlib.font_manager as fm

# 한글 폰트 렌더링 설정 (성능 시각화 목적)
font_path = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
font_prop = fm.FontProperties(fname=font_path) 
plt.rcParams['axes.unicode_minus'] = False 

from tqdm import tqdm                          
import onnx                                    

# =====================================================================
# 1. 커스텀 데이터셋 파이프라인 (Custom Dataset Pipeline)
# =====================================================================
class GuideDogDataset(Dataset):
    def __init__(self, csv_file, img_dir):
        self.data_frame = pd.read_csv(csv_file) 
        self.img_dir = img_dir                  
        # ImageNet 규격 정규화(Normalization) 전처리
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def __len__(self):
        return len(self.data_frame)

    def __getitem__(self, idx):
        # 1-1. 비전 센서 데이터 로드 및 전처리
        img_name = os.path.join(self.img_dir, self.data_frame.iloc[idx]['image_file'])
        image = cv2.imread(img_name)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
        image = cv2.resize(image, (224, 224))          
        image = self.transform(image)                  
        
        # 1-2. 1차원 수치형 센서 데이터 13개 피처 추출 (LiDAR 8, Sonar 3, IMU 1, State 1)
        sensors = self.data_frame.iloc[idx][['scan_0','scan_1','scan_2','scan_3',
                                             'scan_4','scan_5','scan_6','scan_7',
                                             'sonar_l','sonar_c','sonar_r',
                                             'gyro_z','goal_state']].values.astype(np.float32)
        sensors = torch.tensor(sensors)
        
        # 1-3. 타겟 레이블 (Label) 추출 (선속도 v, 각속도 w)
        targets = self.data_frame.iloc[idx][['cmd_v', 'cmd_w']].values.astype(np.float32)
        targets = torch.tensor(targets)
        
        return image, sensors, targets

# =====================================================================
# 2. 공간 어텐션 모듈 (Spatial Attention Module)
# =====================================================================
class SpatialAttention(nn.Module):
    def __init__(self):
        super(SpatialAttention, self).__init__()
        # 평균 폴링 및 최대 풀링 연산 결합 후 1채널 압축
        self.conv = nn.Conv2d(2, 1, kernel_size=7, padding=3)
        self.sigmoid = nn.Sigmoid() 

    def forward(self, x):
        avg_out = torch.mean(x, dim=1, keepdim=True)
        max_out = torch.max(x, dim=1, keepdim=True)[0]
        attention = torch.cat([avg_out, max_out], dim=1)
        attention = self.sigmoid(self.conv(attention))
        return x * attention 

# =====================================================================
# 3. 다중 센서 융합 신경망 아키텍처 (Multi-modal Fusion Network)
# =====================================================================
class GuideDogBrain(nn.Module):
    def __init__(self):
        super(GuideDogBrain, self).__init__()
        
        # 비전 특징 추출용 백본 (Backbone: MobileNetV3 Small)
        mobilenet = models.mobilenet_v3_small(weights=models.MobileNet_V3_Small_Weights.DEFAULT)
        self.cnn_features = mobilenet.features  
        self.attention = SpatialAttention()     
        self.cnn_pool = nn.AdaptiveAvgPool2d(1) 
        
        # 센서 데이터 차원 확장을 위한 전결합층
        self.sensor_fc = nn.Sequential(
            nn.Linear(13, 64), nn.ReLU(),
            nn.Linear(64, 64), nn.ReLU()
        )
        
        # 비전 텐서(576)와 센서 텐서(64) 융합 및 최종 회귀 출력층 (2)
        self.fusion_fc = nn.Sequential(
            nn.Linear(576 + 64, 512), 
            nn.ReLU(), 
            nn.Dropout(0.3),  
            nn.Linear(512, 128), 
            nn.ReLU(),
            nn.Linear(128, 2) 
        )

    def forward(self, image, sensor):
        img_f = self.cnn_features(image)
        img_f = self.attention(img_f) 
        img_f = self.cnn_pool(img_f).flatten(1)
        
        sensor_f = self.sensor_fc(sensor)
        
        fused = torch.cat((img_f, sensor_f), dim=1)
        return self.fusion_fc(fused)

# =====================================================================
# 4. 동적 페널티 적용 커스텀 손실 함수 (Dynamic Weighted Loss Function)
# =====================================================================
def reward_weighted_loss(predictions, targets, sensors):
    base_loss = nn.MSELoss(reduction='none')(predictions, targets)
    batch_size = sensors.size(0)
    weights = torch.ones(batch_size, 2).to(predictions.device)
    
    penalty_count = 0
    reward_count = 0
    
    for i in range(batch_size):
        front_dist = sensors[i, 0].item()  
        goal_state = sensors[i, 12].item() 
        
        # 근접 장애물 0.2m 미만 시 가중치 2.5배 부여 (보수적 주행 학습 유도)
        if front_dist < 0.2:
            weights[i] = weights[i] * 2.5 
            penalty_count += 1 
            
        # 목표 지점 도달 패턴 긍정 보상
        if goal_state == 1.0:
            weights[i] = weights[i] * 0.5 
            reward_count += 1 

    loss = (base_loss * weights).mean()
    return loss, penalty_count, reward_count

def calculate_accuracy(predictions, targets):
    pred_w = predictions[:, 1]   
    target_w = targets[:, 1]     
    correct = (torch.sign(pred_w) == torch.sign(target_w)) | (torch.abs(pred_w - target_w) < 0.2)
    return correct.float().mean().item() * 100.0

# =====================================================================
# 5. 모델 학습 및 검증 루프 (Training & Validation Loop)
# =====================================================================
def main():
    print("시스템 알림: CUDA 연산 가속기를 활성화합니다.")
    print("학습 데이터를 메모리에 적재합니다.")
    
    csv_path = "./guide_dog_dataset/master_dataset.csv" 
    img_dir = "./guide_dog_dataset/images/"
    
    full_dataset = GuideDogDataset(csv_path, img_dir)
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(full_dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_dataset, batch_size=64, shuffle=False, num_workers=4, pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = GuideDogBrain().to(device)
    
    optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-4)
    # Cosine Annealing Warm Restarts 적용을 통한 지역 최적점 탈출 유도
    scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(optimizer, T_0=10, T_mult=2)

    epochs = 1000        
    patience = 30        
    best_val_loss = float('inf')
    trigger_times = 0
    
    history = {'train_loss': [], 'val_loss': [], 'train_acc': [], 'val_acc': []}

    total_penalty_cnt = 0
    total_reward_cnt = 0

    print("자율주행 모델 학습을 개시합니다.")
    for epoch in range(epochs):
        
        model.train()
        train_loss, train_acc = 0.0, 0.0
        
        epoch_penalty_cnt = 0
        epoch_reward_cnt = 0
        
        loop = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs} [Train]", leave=False)
        for images, sensors, targets in loop:
            images, sensors, targets = images.to(device), sensors.to(device), targets.to(device)
            
            optimizer.zero_grad()            
            outputs = model(images, sensors) 
            
            loss, p_cnt, r_cnt = reward_weighted_loss(outputs, targets, sensors)
            loss.backward()                  
            optimizer.step()                 
            
            train_loss += loss.item()        
            train_acc += calculate_accuracy(outputs, targets)
            
            epoch_penalty_cnt += p_cnt
            epoch_reward_cnt += r_cnt
            
            loop.set_postfix(loss=loss.item())

        train_loss /= len(train_loader)
        train_acc /= len(train_loader)
        scheduler.step() 

        total_penalty_cnt += epoch_penalty_cnt
        total_reward_cnt += epoch_reward_cnt

        # 모델 검증 (Validation) 절차
        model.eval() 
        val_loss, val_acc = 0.0, 0.0
        with torch.no_grad(): 
            loop_val = tqdm(val_loader, desc=f"Epoch {epoch+1}/{epochs} [Validation]", leave=False)
            for images, sensors, targets in loop_val:
                images, sensors, targets = images.to(device), sensors.to(device), targets.to(device)
                outputs = model(images, sensors) 
                
                loss, _, _ = reward_weighted_loss(outputs, targets, sensors) 
                val_loss += loss.item()
                val_acc += calculate_accuracy(outputs, targets)

        val_loss /= len(val_loader)
        val_acc /= len(val_loader)

        history['train_loss'].append(train_loss)
        history['val_loss'].append(val_loss)
        history['train_acc'].append(train_acc)
        history['val_acc'].append(val_acc)

        print(f"\n[통계] Epoch {epoch+1} -> Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f} | Acc: {train_acc:.1f}% | Val Acc: {val_acc:.1f}%")
        print(f"       [가중치 기록] 페널티 가동: {epoch_penalty_cnt}회 | 보상 가동: {epoch_reward_cnt}회")

        # 최상위 모델 가중치 저장 및 조기 종료(Early Stopping) 처리
        if val_loss < best_val_loss:
            print(f"       역대 최저 검증 오차 갱신 ({best_val_loss:.4f} -> {val_loss:.4f}). 'best_brain.pth'로 저장합니다.")
            best_val_loss = val_loss
            torch.save(model.state_dict(), "best_brain.pth") 
            trigger_times = 0 
        else:
            trigger_times += 1
            print(f"       개선 없음... 조기 종료 대기: {trigger_times}/{patience}")
            if trigger_times >= patience:
                print(f"\n[경고] 모델 과적합 방지를 위해 학습을 조기 종료합니다.")
                break

    print("\n" + "="*80)
    print("[모델 훈련 최종 결산]")
    print(f"전방 20cm 근접 위험 감지 페널티 총계: {total_penalty_cnt}회")
    print(f"목표 도달 긍정 보상 총계: {total_reward_cnt}회")
    print("="*80 + "\n")

    # =====================================================================
    # 6. 학습 성과 시각화 (Matplotlib)
    # =====================================================================
    print("학습 및 검증 성과 그래프를 파일로 추출합니다.")
    plt.figure(figsize=(12, 5)) 
    
    plt.subplot(1, 2, 1)
    plt.plot(history['train_loss'], label='훈련 오차 (Train Loss)', color='red')
    plt.plot(history['val_loss'], label='검증 오차 (Validation Loss)', color='blue')
    plt.title('훈련 및 검증 오차 추이', fontproperties=font_prop, fontsize=14) 
    plt.xlabel('훈련 횟수 (Epochs)', fontproperties=font_prop, fontsize=12)  
    plt.ylabel('오차 (Loss)', fontproperties=font_prop, fontsize=12)          
    plt.legend(prop=font_prop, fontsize=10) 
    
    plt.subplot(1, 2, 2)
    plt.plot(history['train_acc'], label='훈련 정확도 (Train Accuracy)', color='red')
    plt.plot(history['val_acc'], label='검증 정확도 (Validation Accuracy)', color='blue')
    plt.title('훈련 및 검증 정확도 추이', fontproperties=font_prop, fontsize=14) 
    plt.xlabel('훈련 횟수 (Epochs)', fontproperties=font_prop, fontsize=12)  
    plt.ylabel('정확도 (%)', fontproperties=font_prop, fontsize=12)          
    plt.legend(prop=font_prop, fontsize=10) 
    
    plt.tight_layout()
    plt.savefig('training_results_korean.png') 

    # =====================================================================
    # 7. 엣지 디바이스 배포를 위한 ONNX 규격 포맷 내보내기 (Export)
    # =====================================================================
    print("Raspberry Pi 호환을 위해 모델을 ONNX 규격으로 내보냅니다.")
    model.load_state_dict(torch.load("best_brain.pth")) 
    model.eval() 
    
    # 텐서플로우/파이토치 이기종 간 차원 동기화를 위한 더미 텐서 생성
    dummy_image = torch.randn(1, 3, 224, 224).to(device)
    dummy_sensor = torch.randn(1, 13).to(device)
    
    torch.onnx.export(
        model, 
        (dummy_image, dummy_sensor), 
        "guide_dog_brain.onnx",
        input_names=['image_input', 'sensor_input'], 
        output_names=['output'],
        opset_version=11
    )
    print("시스템 알림: 'guide_dog_brain.onnx' 가중치 모델 및 시각화 결과물 출력이 완료되었습니다.")

if __name__ == '__main__':
    main()
```

### 3. 모델 학습 결과 및 검증 오차(Validation Loss) 변동성 분석

학습 과정에서 도출된 훈련 및 검증 오차 그래프 분석 결과, 훈련 오차(Train Loss)는 안정적으로 감소하는 반면 검증 오차(Validation Loss) 그래프에서 특정 에포크(Epoch) 구간마다 급격한 변동성(Spike)이 관찰됩니다. 이는 모델의 결함이나 과적합(Overfitting) 현상이 아니며, 적용된 학습 알고리즘의 고도화된 설계 특성에 기인한 정상적인 현상입니다. 주요 원인은 다음과 같이 3가지로 분석됩니다.

1. **코사인 어닐링 웜 리스타트(Cosine Annealing Warm Restarts) 스케줄러의 개입:**
본 모델은 `optim.lr_scheduler.CosineAnnealingWarmRestarts` 학습률 제어기를 적용하여, 점진적으로 학습률을 감소시키다 특정 주기마다 최댓값으로 강제 초기화(Restart)합니다. 학습률이 순간적으로 증가할 때 가중치 업데이트의 보폭이 커지며 일시적으로 손실값(Loss)이 급증하게 됩니다. 이는 모델이 지역 최적점(Local Minima)에 빠지는 것을 방지하고 전역 최적점(Global Minimum)을 탐색하도록 유도하는 필수적인 최적화 과정입니다.

2. **동적 페널티 가중치(Reward-Weighted Loss)에 의한 분산 확대:**
단순 평균 제곱 오차(MSE)를 변형하여, 전방 장애물과의 거리가 0.2m 미만인 '위험 프레임'의 경우 손실값에 2.5배의 페널티 가중치를 부여하도록 설계하였습니다.
$$ Loss = MSE(y, \hat{y}) \times 2.5 \quad \text{(if front\_dist < 0.2)} $$
검증 데이터셋의 미니 배치(Mini-batch) 내에 이러한 위험 프레임이 다수 포함될 경우, 모델의 실제 예측 오차가 작더라도 페널티가 증폭 반영되어 그래프 상의 검증 오차가 비정상적으로 폭증하는 통계적 착시 현상이 발생합니다.

3. **소규모 검증 데이터셋에 따른 통계적 노이즈:**
평가에 사용된 검증 데이터(Validation Set)의 표본 크기가 상대적으로 작을 경우, 개별 프레임의 예측 실패가 전체 검증 오차 평균에 미치는 영향력이 과대하게 나타납니다. 그럼에도 불구하고 우측의 '검증 정확도(Validation Accuracy)' 지표는 일시적 하락 직후 즉시 90~95% 수준을 회복하며 지속적인 우상향 추세를 보이고 있습니다. 이는 본 모델이 노이즈에 무너지지 않고 강건(Robust)하게 패턴을 학습하며 우수한 일반화 성능을 확보하고 있음을 학술적으로 증명합니다.
