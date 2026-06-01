// =====================================================================
// 파일명: aicar_arduino_master.ino
// 목적: 4륜 구동 모터 방향 제어, 초음파 센서 기반 장애물 회피, MPU-6050 3축 자이로 센서 데이터 취득 및 직렬 통신
// =====================================================================

#include <AFMotor.h>
#include <NewPing.h>
#include <Wire.h>

#define MAX_DISTANCE 200 // 초음파 센서 최대 유효 측정 거리 (단위: cm)
#define NORMAL_SPEED 210 // 평시 직진 주행 모터 PWM 속도 제어값
#define TURN_SPEED 190   // 제자리 회전 시 슬립(Slip) 방지를 위한 하향 조정된 모터 PWM 속도 제어값

// 초음파 센서 객체 배열 초기화 (좌측, 중앙, 우측) - 핀 충돌(Pin Conflict) 회피 설계 적용
NewPing sonarLeft(A0, A1, MAX_DISTANCE);   
NewPing sonarCenter(A2, A3, MAX_DISTANCE); 
NewPing sonarRight(9, 10, MAX_DISTANCE);   

// L293D 모터 드라이버 쉴드 제어 객체 초기화 (구동륜 4개 채널 할당)
AF_DCMotor motorRR(1); 
AF_DCMotor motorFR(2);
AF_DCMotor motorFL(3); 
AF_DCMotor motorRL(4);

char current_cmd = 's'; // 상위 제어기(Raspberry Pi)로부터 수신된 현재 이동 명령 버퍼
unsigned long last_cmd_time = 0, last_ping_time = 0, last_imu_time = 0, last_sonar_pub_time = 0;
int current_sensor = 0; 
int distL = MAX_DISTANCE, distC = MAX_DISTANCE, distR = MAX_DISTANCE;
float gyro_z_offset = 0.0; // 자이로 센서 누적 오차(Drift) 보정을 위한 영점 오프셋 변수

void setup() {
  Serial.begin(115200);  // 상위 제어기와의 직렬 통신 보드레이트(Baudrate) 설정
  Serial.setTimeout(10); // 직렬 통신 수신 대기 시간(Timeout)을 10ms로 최소화하여 응답성 향상
  Wire.begin(); 
  
  // MPU-6050 센서 초기화 및 I2C 통신 설정
  // 디지털 저역통과필터(DLPF) 대역폭을 10Hz로 설정하여 기계적 진동 노이즈 억제
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x80); Wire.endTransmission(true); delay(100); 
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); delay(100); 
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(true); 
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true); 

  set_speed_all(NORMAL_SPEED); 
  stop_motors(); 

  // 시스템 부팅 단계: 2초간 500회의 자이로 센서 샘플링을 통한 영점 캘리브레이션(Calibration) 수행
  long gyro_sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
    if(Wire.available() >= 2) gyro_sum += (Wire.read() << 8 | Wire.read()); 
    delay(4);
  }
  gyro_z_offset = (gyro_sum / 500.0) / 131.0; 
}

void loop() {
  unsigned long current_time = millis(); 

  // 1. 관성측정장치(IMU) 제어: 50ms(20Hz) 주기로 Z축 각속도(Angular Velocity) 데이터를 상위 제어기로 전송
  if (current_time - last_imu_time >= 50) {
    last_imu_time = current_time;
    Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true); 
    if(Wire.available() >= 2) {
      float gyro_z = ((Wire.read() << 8 | Wire.read()) / 131.0) - gyro_z_offset; 
      Serial.print("I,"); Serial.print(gyro_z); Serial.print("\n"); 
    }
  }

  // 2. 초음파 센서 제어: 30ms 주기로 3개의 센서를 순차적(Round-Robin)으로 트리거하여 음파 간섭(Cross-talk) 방지
  if (current_time - last_ping_time >= 30) {
    last_ping_time = current_time;
    if (current_sensor == 0) { distL = sonarLeft.ping_cm(); if (distL == 0) distL = MAX_DISTANCE; current_sensor = 1; } 
    else if (current_sensor == 1) { distC = sonarCenter.ping_cm(); if (distC == 0) distC = MAX_DISTANCE; current_sensor = 2; } 
    else if (current_sensor == 2) { distR = sonarRight.ping_cm(); if (distR == 0) distR = MAX_DISTANCE; current_sensor = 0; }
  }

  // 3. 거리 데이터 전송: 100ms(10Hz) 주기로 갱신된 초음파 거리 데이터를 상위 제어기로 일괄 보고
  if (current_time - last_sonar_pub_time >= 100) {
    last_sonar_pub_time = current_time;
    Serial.print("U,"); Serial.print(distL); Serial.print(","); Serial.print(distC); Serial.print(","); Serial.print(distR); Serial.print("\n");
  }

  // 4. 긴급 제동(Emergency Brake) 페일세이프(Failsafe) 로직
  // 전진 기동 중 전방 5cm 이내, 또는 측면 2cm 이내 장애물 탐지 시 상위 제어기의 명령을 하드웨어 단에서 강제 무시
  bool emergency_brake = false; 
  if (current_cmd == 'f' || current_cmd == 'q' || current_cmd == 'e') { 
    if ((distC > 0 && distC <= 5) || (distL > 0 && distL <= 2) || (distR > 0 && distR <= 2)) emergency_brake = true;
  }

  // 5. 상위 제어기 명령 수신 및 통신 두절(Timeout) 시 강제 정지 보호 로직
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); 
    if (data.startsWith("C,")) { current_cmd = data.charAt(2); last_cmd_time = current_time; }
  }
  if (current_time - last_cmd_time > 1000) { current_cmd = 's'; } 

  // 6. 수신 명령 및 안전 상태에 따른 구동륜 8방향 벡터 제어 수행
  if (emergency_brake) { stop_motors(); } 
  else {
    if (current_cmd == 'f') { set_speed_all(NORMAL_SPEED); move_forward_dir(); }
    else if (current_cmd == 'b') { set_speed_all(NORMAL_SPEED); move_backward_dir(); }
    else if (current_cmd == 'q') { set_speed_all(NORMAL_SPEED); move_forward_left(); } // 좌측 전방 대각선 주행
    else if (current_cmd == 'e') { set_speed_all(NORMAL_SPEED); move_forward_right(); } // 우측 전방 대각선 주행
    else if (current_cmd == 'l') { set_speed_all(TURN_SPEED); rotate_left(); } // 제자리 좌회전(Skid Turn)
    else if (current_cmd == 'r') { set_speed_all(TURN_SPEED); rotate_right(); } // 제자리 우회전(Skid Turn)
    else { stop_motors(); } 
  }
}

// === 모터 구동 방향 및 PWM 제어 서브루틴(Sub-routine) ===
void set_speed_all(int speed) { motorRR.setSpeed(speed); motorFR.setSpeed(speed); motorFL.setSpeed(speed); motorRL.setSpeed(speed); }
void move_forward_dir() { motorRR.run(FORWARD); motorFR.run(FORWARD); motorFL.run(FORWARD); motorRL.run(FORWARD); }
void move_backward_dir() { motorRR.run(BACKWARD); motorFR.run(BACKWARD); motorFL.run(BACKWARD); motorRL.run(BACKWARD); }
void rotate_left() { motorRR.run(FORWARD); motorFR.run(FORWARD); motorFL.run(BACKWARD); motorRL.run(BACKWARD); } 
void rotate_right() { motorRR.run(BACKWARD); motorFR.run(BACKWARD); motorFL.run(FORWARD); motorRL.run(FORWARD); }
void stop_motors() { motorRR.run(RELEASE); motorFR.run(RELEASE); motorFL.run(RELEASE); motorRL.run(RELEASE); }
void move_forward_left() { motorRR.run(FORWARD); motorFR.run(FORWARD); motorFL.run(RELEASE); motorRL.run(RELEASE); } 
void move_forward_right() { motorRR.run(RELEASE); motorFR.run(RELEASE); motorFL.run(FORWARD); motorRL.run(FORWARD); }

