# 자율주행 안내견 로봇(AI Guide Dog) ROS 2 Nav2 시스템 통합 백서

**작성일자:** 2026년 4월 23일
**프로젝트 책임자:** J2W
**수석 엔지니어:** J2W

본 문서는 라즈베리파이(Raspberry Pi)와 아두이노(Arduino)를 연동한 4륜 스키드 스티어링(Skid-Steering) 기반 모바일 로봇을 ROS 2 Humble 환경에서 제어하고, RViz2를 활용한 2D SLAM 맵핑 및 Nav2 자율주행, 그리고 자율 순찰 기능을 구현한 시스템 엔지니어링 기록입니다. 나아가 딥러닝 비전 및 거대 언어 모델(LLM)을 융합하는 비전-언어-행동(VLA) 아키텍처 로드맵을 포함합니다.

---

## 제 1장: 시스템 하드웨어 상세 제원

자율주행 내비게이션(Nav2)의 정밀한 제어를 위한 기구학적(Kinematics) 제원 및 컴퓨팅 리소스 사양은 다음과 같습니다.

### 1. 물리적 기구학 제원
* **전장 (Length):** 0.27m (270mm) - Nav2 로봇 점유 영역(Footprint) 설계의 기준 수치입니다.
* **윤거 (Track Width):** 0.15m (150mm) - 좌우 바퀴 중심 간 거리입니다.
* **축거 (Wheelbase):** 0.13m (130mm) - 앞뒤 바퀴 축 간 거리입니다.
* **타이어 직경:** 0.066m (66mm) - 적용된 엔코더 분해능(20 Ticks/Rev) 기준 1틱당 이동 거리는 약 0.01036m로 산출됩니다.
* **슬립 보정 계수 (Slip Factor):** 실내 환경에서의 헛돌음 현상을 보정하기 위해 `SLIP_FACTOR = 0.95`를 적용하였습니다.

### 2. 컴퓨팅 및 인공지능 코어
* **메인 프로세서 (Main Processor):** Raspberry Pi 4 (4GB RAM) - Ubuntu OS 기반 ROS 2 Humble 미들웨어를 구동하며, SLAM 및 Nav2 연산을 전담합니다.
* **하위 제어기 (Microcontroller):** Arduino Uno + L293D 모터 드라이버 쉴드 - 실시간 모터 PWM 제어 및 하드웨어 센서 인터럽트를 처리합니다.
* **AI 학습 서버 (AI Server):** NVIDIA GeForce RTX 4070 12GB VRAM 탑재 워크스테이션 - 딥러닝 모델(CNN, LLM)의 오프라인 학습을 담당합니다.

### 3. 센서 시스템
* **2D 라이다 (LiDAR):** SLLidar (지상고 15cm) - 2D 점군(Point Cloud) 데이터 기반 지도 생성 및 장애물 회피에 사용됩니다.
* **관성 측정 장치 (IMU):** MPU-6050 - 제자리 회전 시 발생하는 스키드 스티어링의 마찰 오차 및 방위각 보정에 활용됩니다.
* **초음파 센서 (Ultrasonic):** HC-SR04 배열 3개 (지상고 3cm) - 라이다 스캔 평면 하단의 사각지대 장애물 탐지 및 긴급 제동(Emergency Brake) 목적으로 운용됩니다.
* **광학식 엔코더 (Encoder):** 각 모터에 장착된 4개의 광학식 엔코더(20 Ticks/Rev)를 통해 휠 오도메트리를 산출합니다.
* **비전 센서 (Camera):** Raspberry Pi Camera Rev 1.3 - 향후 비전 기반 행동 복제(Behavioral Cloning) 학습용 데이터 수집에 활용됩니다.

### 4. 독립 전력 공급 시스템
* **배터리 사양:** KLIFE PD-Q2 고속 충전 보조배터리 2기 (10000mAh, 37Wh).
* **공급 출력:** Type-C 5V/3A, 9V/2.22A, 12V/1.67A.
* **전력망 분리 설계:** 메인 프로세서(라즈베리파이)와 구동부(아두이노)의 전력망을 물리적으로 분리하여, 모터 과부하 시 발생하는 전압 강하(Brownout) 및 이로 인한 메인 프로세서 강제 재부팅 현상을 하드웨어적으로 방지하였습니다.

---

## 제 2장: 시스템 통합 및 트러블슈팅(Troubleshooting) 분석

### 이슈 1: Nav2 제어기 강제 종료 (ZeroDivision & Extrapolation Error)
* **증상:** RViz2 환경에서 `2D Pose Estimate`는 정상 동작하나, 목적지(Goal) 설정 시 TF 프레임 연결이 끊어지며 노드가 강제 종료(SIGKILL)되는 현상입니다.
* **원인:**
  1. 오도메트리 노드(`real_odom_publisher.py`) 내에서 변수 초기화 전 하드웨어 인터럽트가 발생하여 유발된 경쟁 상태(Race Condition) 오류입니다.
  2. 로봇 위치 추정의 공분산 행렬(Covariance Matrix)이 누락되어, DWB 로컬 플래너 연산 시 0으로 나누기(ZeroDivision) 오류가 발생했습니다.
* **해결 방안:** 변수 선언부를 GPIO 인터럽트 활성화 이전으로 재배치하고, 확장 칼만 필터(EKF) 설계 기준에 따라 사용하지 않는 축(Z축, Roll, Pitch)의 공분산에 `1e6`의 상수를 할당하여 연산 안정성을 확보했습니다.

### 이슈 2: RViz2 오도메트리 회전 오차 (Phantom Rotation)
* **증상:** 물리적 로봇은 정지 상태이나 RViz2 상에서 로봇의 TF 프레임이 지속적으로 회전하는 현상입니다.
* **원인:** MPU-6050 센서의 초기 자전 오차(Gyro Bias)에 대한 영점 조절(Calibration)이 수행되지 않았습니다.
* **해결 방안:** 하위 제어기 초기화(`setup()`) 단계에서 2초간 500회의 데이터를 샘플링하여 오차(`gyro_z_offset`)를 산출하고, 실시간 계측 데이터에서 이를 차감하는 영점 조절 알고리즘을 적용했습니다.

### 이슈 3: MPU-6050 센서 통신 데드락 및 하드웨어 결함 진단
* **증상:** I2C 통신 주소(`0x68`)는 식별되나 데이터가 갱신되지 않거나 노이즈 값만 출력되는 현상입니다.
* **원인 분석 및 해결 방안:** 고주파 노이즈 억제를 위해 디지털 저역 통과 필터(DLPF, 10Hz)를 활성화하고 하드웨어 강제 초기화(`0x80`)를 수행하였으나, 시리얼 모니터링 결과 Z축의 지구 중력(1G) 계측이 불가능한 물리적 MEMS 구조 결함으로 판정되어 센서를 신규 교체하여 문제를 해결했습니다.

---

## 제 3장: 최종 자율주행 시스템 소스 코드

### 1. 하위 제어기 메인 펌웨어 (`aicar_arduino_master.ino`)
하드웨어 노이즈 필터링 및 MPU-6050 자동 영점 캘리브레이션 기능이 포함된 아두이노 통합 제어 코드입니다.

```cpp
/* * =========================================================================
 * 모바일 로봇 하위 제어기 통합 펌웨어: 모터 방향 제어, 초음파 센싱, IMU 계측
 * ========================================================================= */
#include <AFMotor.h> 
#include <NewPing.h> 
#include <Wire.h>    

#define MAX_DISTANCE 200 // 초음파 센서 유효 측정 한계 (cm)
#define NORMAL_SPEED 210 // 직진 주행 시 기본 PWM 듀티비
#define TURN_SPEED 240   // 제자리 회전 시 슬립 방지를 위한 보정 PWM 듀티비

// 다채널 초음파 센서 객체 선언 (좌측, 중앙, 우측)
NewPing sonarLeft(A0, A1, MAX_DISTANCE);   
NewPing sonarCenter(A2, A3, MAX_DISTANCE); 
NewPing sonarRight(9, 10, MAX_DISTANCE);   

// 4륜 구동용 DC 모터 채널 할당
AF_DCMotor motorRR(1); AF_DCMotor motorFR(2);
AF_DCMotor motorFL(3); AF_DCMotor motorRL(4);

char current_cmd = 's'; // 수신된 제어 명령 버퍼 (초기값: 정지)
unsigned long last_cmd_time = 0;   // 통신 연결 상태 확인용 타임스탬프
unsigned long last_ping_time = 0;  // 초음파 센싱 주기 제어용 타임스탬프
unsigned long last_imu_time = 0;   // IMU 계측 주기 제어용 타임스탬프

int current_sensor = 0; // 초음파 센서 간섭(Cross-talk) 방지용 라운드 로빈 인덱스
int distL = MAX_DISTANCE, distC = MAX_DISTANCE, distR = MAX_DISTANCE; 

float gyro_z_offset = 0.0; // 자이로 센서 Z축 영점 오프셋

void setup() {
  Serial.begin(115200);  // 상위 제어기와의 직렬 통신 보드레이트 설정
  Serial.setTimeout(10); // 수신 대기시간 최소화
  Wire.begin(); 
  
  // MPU-6050 초기화 및 디지털 저역 통과 필터(DLPF, 10Hz) 활성화
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x80); Wire.endTransmission(true); delay(100); 
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); delay(100); 
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(true); 
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true); 

  set_speed_all(NORMAL_SPEED); 
  stop_motors(); 

  // MPU-6050 Z축 자이로 영점 자동 캘리브레이션 (초기 2초간 정지 상태 유지 필요)
  long gyro_sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
    if(Wire.available() >= 2) {
      int16_t gz = Wire.read() << 8 | Wire.read();
      gyro_sum += gz; 
    }
    delay(4);
  }
  gyro_z_offset = (gyro_sum / 500.0) / 131.0; 
}

void loop() {
  unsigned long current_time = millis(); 

  // [IMU 계측] 50ms(20Hz) 주기로 Z축 각속도 추출 및 오차 보정
  if (current_time - last_imu_time >= 50) {
    last_imu_time = current_time;
    Wire.beginTransmission(0x68);
    Wire.write(0x47); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true); 
    
    if(Wire.available() >= 2) {
      int16_t gz = Wire.read() << 8 | Wire.read(); 
      float raw_gyro_z = gz / 131.0; 
      float gyro_z = raw_gyro_z - gyro_z_offset; 
      Serial.print("I,"); Serial.print(gyro_z); Serial.print("\n"); 
    }
  }

  // [초음파 센싱] 30ms 주기로 순차적 계측 수행 (간섭 방지)
  if (current_time - last_ping_time >= 30) {
    last_ping_time = current_time;
    if (current_sensor == 0) { distL = sonarLeft.ping_cm(); if (distL == 0) distL = MAX_DISTANCE; current_sensor = 1; } 
    else if (current_sensor == 1) { distC = sonarCenter.ping_cm(); if (distC == 0) distC = MAX_DISTANCE; current_sensor = 2; } 
    else if (current_sensor == 2) { distR = sonarRight.ping_cm(); if (distR == 0) distR = MAX_DISTANCE; current_sensor = 0; }
  }

  // [페일세이프] 전진 기동 중 근접 장애물 탐지 시 하드웨어 긴급 제동
  bool emergency_brake = false; 
  if (current_cmd == 'f' || current_cmd == 'F') { 
    if ((distC > 0 && distC <= 5) || (distL > 0 && distL <= 2) || (distR > 0 && distR <= 2)) {
      emergency_brake = true;
    }
  }

  // [통신 프로토콜] 상위 제어기(ROS 2) 속도 명령 수신
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); 
    if (data.startsWith("C,")) { current_cmd = data.charAt(2); last_cmd_time = current_time; }
  }
  
  // 통신 연결 1초 이상 단절 시 안전을 위해 강제 정지
  if (current_time - last_cmd_time > 1000) { current_cmd = 's'; } 

  // [모터 구동 제어] 수신 명령에 따른 방향 및 속도 제어
  if (emergency_brake) { stop_motors(); } 
  else {
    if (current_cmd == 'f') { set_speed_all(NORMAL_SPEED); move_forward_dir(); }
    else if (current_cmd == 'l') { set_speed_all(TURN_SPEED); rotate_left(); }
    else if (current_cmd == 'r') { set_speed_all(TURN_SPEED); rotate_right(); }
    else { stop_motors(); } 
  }
}

// === 모터 구동 서브루틴 ===
void set_speed_all(int speed) { motorRR.setSpeed(speed); motorFR.setSpeed(speed); motorFL.setSpeed(speed); motorRL.setSpeed(speed); }
void move_forward_dir() { motorRR.run(FORWARD); motorFR.run(FORWARD); motorFL.run(FORWARD); motorRL.run(FORWARD); }
void rotate_left() { motorRR.run(FORWARD); motorFR.run(FORWARD); motorFL.run(BACKWARD); motorRL.run(BACKWARD); } 
void rotate_right() { motorRR.run(BACKWARD); motorFR.run(BACKWARD); motorFL.run(FORWARD); motorRL.run(FORWARD); }
void stop_motors() { motorRR.run(RELEASE); motorFR.run(RELEASE); motorFL.run(RELEASE); motorRL.run(RELEASE); }
```

### 2. 제어 명령 변환 브리지 노드 (`motor_bridge.py`)
디바이스 포트 간 충돌을 방지하고 ROS 2 토픽을 시리얼 프로토콜로 변환하는 브리지 노드입니다.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        # 상위 제어기의 제어 속도(cmd_vel) 구독
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # 하위 제어기로부터 수신된 MPU-6050 계측 데이터 발행
        self.imu_pub = self.create_publisher(Float32, 'gyro_z', 10)
        
        try: 
            # 아두이노 통신을 위한 직렬 포트 정적 할당
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("하위 제어기 통신 포트(ACM0) 연결 성공")
        except Exception as e: 
            self.get_logger().error(f"통신 연결 오류: {e}")
            
        self.timer = self.create_timer(0.02, self.read_serial) 

    def cmd_vel_callback(self, msg):
        # 수신된 Twist 벡터 데이터를 단일 문자 명령어로 매핑
        cmd = 's' 
        if msg.linear.x > 0.001: cmd = 'f'       
        elif msg.angular.z > 0.001: cmd = 'l'    
        elif msg.angular.z < -0.001: cmd = 'r'   

        send_data = f"C,{cmd}\n"
        if hasattr(self, 'ser'): self.ser.write(send_data.encode()) 

    def read_serial(self):
        # 직렬 통신 데이터 파싱 및 ROS 2 메시지 발행
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("I,"): 
                    msg = Float32()
                    msg.data = float(line.split(',')[1])
                    self.imu_pub.publish(msg) 
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. 센서 퓨전 오도메트리 발행 노드 (`real_odom_publisher.py`)
차량의 기구학적 제원과 슬립 보정, 그리고 EKF 연산을 위한 공분산 행렬이 반영된 오도메트리 노드입니다.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import math
import RPi.GPIO as GPIO 

class RealOdomPublisher(Node):
    def __init__(self):
        super().__init__('real_odom_publisher')
        
        # 하드웨어 인터럽트 연결을 위한 GPIO 핀 매핑
        self.ENC_FL = 17; self.ENC_FR = 27; self.ENC_RL = 22; self.ENC_RR = 23
        
        # 타이어 직경(66mm) 및 스키드 스티어링 슬립 보정 계수 적용
        WHEEL_DIAMETER = 0.066 
        TICKS_PER_REV = 20
        self.METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV 
        self.SLIP_FACTOR = 0.95 
        
        # 경쟁 상태(Race Condition) 오류 방지를 위한 변수 사전 선언
        self.ticks = 0
        self.is_moving_forward = False 
        self.x = 0.0; self.y = 0.0; self.th = 0.0; self.vth = 0.0 
        self.last_time = self.get_clock().now()
        
        # GPIO 핀 활성화 및 풀업 저항 적용
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ENC_FL, self.ENC_FR, self.ENC_RL, self.ENC_RR], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.ENC_FL, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_FR, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_RL, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_RR, GPIO.BOTH, callback=self.encoder_cb)

        # ROS 2 퍼블리셔 및 서브스크라이버 초기화
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Float32, 'gyro_z', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) 
        
        self.timer = self.create_timer(0.05, self.publish_odom)
        self.get_logger().info("센서 퓨전 오도메트리 노드가 활성화되었습니다.")

    def encoder_cb(self, channel):
        # 전진 기동 시 펄스 데이터 누적
        if self.is_moving_forward: self.ticks += 1 

    def cmd_vel_callback(self, msg):
        # 주행 방향 상태 갱신
        self.is_moving_forward = True if msg.linear.x > 0.001 else False

    def imu_callback(self, msg):
        # 자이로 센서 노이즈 필터링 및 단위 변환 적용
        self.vth = msg.data * (math.pi / 180.0) if abs(msg.data) > 0.1 else 0.0 

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        
        # 4륜 구동 엔코더 평균 펄스 및 슬립 계수를 반영한 선속도 연산
        delta_dist = ((self.ticks / 4.0) * self.METERS_PER_TICK) * self.SLIP_FACTOR
        self.ticks = 0 
        vx = delta_dist / dt if dt > 0 else 0.0 
        
        # 위치 벡터(X, Y) 연산
        delta_x = delta_dist * math.cos(self.th)
        delta_y = delta_dist * math.sin(self.th)
        delta_th = self.vth * dt
        
        self.x += delta_x; self.y += delta_y; self.th += delta_th
        
        # odom -> base_link 프레임 TF 변환 발행
        t = TransformStamped()
        t.header.stamp = current_time.to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'         
        t.transform.translation.x = self.x; t.transform.translation.y = self.y; t.transform.translation.z = 0.0
        
        q = self.euler_to_quaternion(0, 0, self.th) 
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t) 
        
        # Odometry 메시지 생성 및 발행
        odom = Odometry()
        odom.header.stamp = current_time.to_msg(); odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]; odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx; odom.twist.twist.angular.z = self.vth
        
        # EKF 연산 안정성을 위한 절대 공분산 행렬(Covariance Matrix) 할당
        odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
        odom.twist.covariance = odom.pose.covariance
        
        self.odom_pub.publish(odom) 
        self.last_time = current_time

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5); cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5); cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]

def main(args=None):
    rclpy.init(args=args)
    node = RealOdomPublisher()
    try: rclpy.spin(node) 
    finally:
        GPIO.cleanup(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. 자율 순찰 노드 (`robot_cleaner.py`)
라이다(LiDAR) 스캔 데이터를 활용하여 전방 0.5m 이내 장애물 감지 시 충돌을 회피하며 임의 경로를 순찰하는 랜덤 워크(Random Walk) 노드입니다.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotCleaner(Node):
    def __init__(self):
        super().__init__('robot_cleaner')
        # 라이다 스캔 데이터 구독
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # 제어 속도(cmd_vel) 발행
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.get_logger().info("자율 순찰(Random Walk) 모드가 가동되었습니다.")

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        # 전방 30도(좌우 15도) 유효 시야 영역 확보
        front_angles = int(num_ranges * 15 / 360) 
        front_ranges = ranges[:front_angles] + ranges[-front_angles:]
        
        # 라이다 노이즈 영역 제거
        valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
        min_distance = min(valid_ranges) if valid_ranges else 999.0

        # 전방 0.5m 이내 장애물 탐지 시 좌회전 회피 기동
        if min_distance < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.0 
            self.get_logger().info(f"장애물 감지 (이격 거리: {min_distance:.2f}m), 회피 기동 수행")
        else:
            self.twist.linear.x = 0.2 
            self.twist.angular.z = 0.0
        
        self.cmd_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotCleaner()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 제 4장: 컴포넌트별 터미널 실행 명령어

아래의 시스템 명령어는 개별 터미널 환경에서 독립적으로 순차 실행되어야 합니다.

### [공통 사전 절차: 하드웨어 I/O 제어 권한 할당]
디바이스 부팅 후 1회에 한하여 하드웨어 제어 권한을 설정합니다.
```bash
sudo chmod 777 /dev/ttyUSB* /dev/ttyACM* /dev/video0 /dev/gpiomem && sudo chown root:gpio /dev/gpiomem && sudo chmod g+rw /dev/gpiomem
sed -i 's/footprint: .*/footprint: "[[-0.135, -0.075], [-0.135, 0.075], [0.135, 0.075], [0.135, -0.075]]"/g' ~/ros2_ws/my_nav2_params.yaml
```

### 프로세스 A: 카토그래퍼(Cartographer) 기반 2D SLAM 맵핑
수동 원격 조종을 통해 환경을 스캔하고 Occupancy Grid 지도를 생성하는 절차입니다.

```bash
# 1. 라이다 센서 위치 프레임(TF) 정적 할당
source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.15 --roll 0 --pitch 0 --yaw 1.5708 --frame-id base_link --child-frame-id laser

# 2. 센서 노드 실행
source /opt/ros/humble/setup.bash && ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_baudrate:=460800

# 3. 오도메트리 노드 실행
source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/real_odom_publisher.py

# 4. 제어 명령 브리지 노드 실행
source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/motor_bridge.py

# 5. 카토그래퍼(SLAM) 프로세스 실행
source /opt/ros/humble/setup.bash && ros2 launch cartographer_ros cartographer.launch.py

# 6. 텔레오프(Teleop) 노드 실행을 통한 수동 주행 제어
source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 7. 지도 생성 완료 후 데이터 파일 저장
source /opt/ros/humble/setup.bash && cd ~/ros2_ws && ros2 run nav2_map_server map_saver_cli -f my_guide_dog_map
```

### 프로세스 B: RViz2 및 Nav2 기반 자율주행 실행
생성된 지도를 바탕으로 경로 계획(Path Planning) 및 자율주행 기능을 구동하는 절차입니다.

```bash
# 1. 라이다 센서 위치 프레임(TF) 정적 할당
source /opt/ros/humble/setup.bash && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.15 --roll 0 --pitch 0 --yaw 1.5708 --frame-id base_link --child-frame-id laser

# 2. 센서 노드 실행
source /opt/ros/humble/setup.bash && ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_baudrate:=460800

# 3. 오도메트리 노드 실행
source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/real_odom_publisher.py

# 4. 제어 명령 브리지 노드 실행
source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/motor_bridge.py

# 5. 비전 카메라 노드 실행
source /opt/ros/humble/setup.bash && ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"

# 6. 내비게이션(Nav2) 스택 실행
source /opt/ros/humble/setup.bash && cd ~/ros2_ws && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false autostart:=true map:=/home/pi2jw/ros2_ws/my_guide_dog_map.yaml params_file:=/home/pi2jw/ros2_ws/my_nav2_params.yaml

# 7. RViz2 시각화 도구 실행 및 목적지 설정
source /opt/ros/humble/setup.bash && ros2 run rviz2 rviz2 -d my_guide_dog.rviz

# 8. (선택 사항) 자율 순찰 노드 실행
source /opt/ros/humble/setup.bash && python3 ~/ros2_ws/src/robot_cleaner.py
```

---

## 제 5장: 향후 시스템 고도화 로드맵 (비전-언어-행동 VLA 융합)

기존 LiDAR 기반 2D 평면 매핑을 넘어 카메라 비전 시스템과 거대 언어 모델(LLM)을 결합한 통합 VLA 아키텍처 개발 계획입니다.

### Phase 1: 행동 복제(Behavioral Cloning)를 위한 비전 데이터 수집
* **목표:** 동적 장애물에 대한 오퍼레이터의 수동 회피 기동 궤적을 딥러닝 훈련 데이터로 변환합니다.
* **방법론:** `rosbag2`와 `cv_bridge` 패키지를 연동하여 제어 변수(Twist)와 비전 텐서(Image)를 고정밀 타임스탬프로 동기화 및 수집합니다.

### Phase 2: 합성곱 신경망(CNN) 기반 학습 모델 배포
* **목표:** 수집된 시각 데이터를 기반으로 자율적인 회피 판단 모델을 구축합니다.
* **방법론:** 고성능 AI 학습 서버(RTX 4070) 환경에서 PyTorch 기반의 CNN 알고리즘을 훈련하여 종단간(End-to-End) 자율주행 가중치 파일(`.pt`)을 최적화합니다.

### Phase 3: 거대 언어 모델(LLM) 연동 자연어 목표 지시 시스템 구축
* **목표:** 음성 명령을 인지하고 유동적인 자율주행 목표를 생성하는 통합 프레임워크를 개발합니다.
* **방법론 (클라우드/엣지 분산 아키텍처):**
  1. 음성 입력 데이터를 STT(Speech-to-Text) 모듈로 처리하여 텍스트 기반 명령을 획득합니다.
  2. 추론 서버의 EXAONE 등 LLM에 텍스트를 전송하여 자연어의 문맥을 분석합니다.
  3. LLM의 환각(Hallucination) 현상 방지를 위해 모델이 직접 코드를 제어하는 방식을 배제하고, `{"action": "navigate", "target": "water_purifier", "x": 3.5, "y": -1.2}`와 같이 시스템 안정성이 보장된 JSON 포맷의 좌표 지시서를 반환하도록 설계합니다.
  4. 엣지 디바이스(Raspberry Pi)가 수신된 JSON 좌표와 내부의 CNN 기반 비전 주행 모델을 융합하여 목적지 도달 시퀀스를 수행합니다.
