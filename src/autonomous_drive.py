#!/usr/bin/env python3
# =====================================================================
# 모바일 로봇 자율주행 통합 제어 노드 (Autonomous Driving Control Node)
# 딥러닝(MobileNetV3+LSTM) 기반 주행 및 VFH(Vector Field Histogram) 기반 끼임(Stuck) 회피 로직 적용
# =====================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import time
import math

class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')
        
        self.get_logger().info("ONNX 런타임 기반 딥러닝 추론 세션을 초기화합니다.")
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 1 # 엣지 디바이스 리소스 최적화를 위한 단일 스레드 할당
        self.session = ort.InferenceSession("/home/pi2jw/ros2_ws/guide_dog_brain.onnx", sess_options)
        self.bridge = CvBridge()

        # 센서 데이터 저장을 위한 초기 상태 변수 할당
        self.latest_scan = [12.0] * 8
        self.latest_sonar_l = 2.0; self.latest_sonar_c = 2.0; self.latest_sonar_r = 2.0
        self.latest_gyro_z = 0.0
        self.latest_odom_x = 0.0; self.latest_odom_y = 0.0
        self.latest_image = None
        
        # 유한 상태 기계(FSM) 제어 및 방향 추적 변수
        self.current_goal_state = 0.0 
        self.gyro_z_accum_for_uturn = 0.0
        self.last_gyro_time = time.time()

        # 국소 최적점(Stuck) 감지 및 VFH 탈출 제어 변수
        self.is_escaping = False       
        self.escape_end_time = 0.0     
        self.escape_angular_z = 1.5    
        self.stuck_start_time = time.time() 
        self.stuck_check_odom_x = 0.0  
        self.stuck_check_odom_y = 0.0  
        
        # 탈출 완료 후 연속적인 끼임 오판을 방지하기 위한 유예 시간(Cooldown) 설정
        self.escape_cooldown_end = 0.0 

        # ROS 2 토픽 구독자(Subscriber) 선언
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Range, '/sonar_left', self.sonar_l_callback, 10)
        self.create_subscription(Range, '/sonar_center', self.sonar_c_callback, 10)
        self.create_subscription(Range, '/sonar_right', self.sonar_r_callback, 10)
        self.create_subscription(Float32, '/gyro_z', self.gyro_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # 제어 명령 발행자(Publisher) 및 메인 제어 루프 타이머(10Hz) 선언
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop) 
        
        self.get_logger().info("자율주행 제어 노드 및 예외 처리 로직이 성공적으로 활성화되었습니다.")

    def scan_callback(self, msg):
        # 2D 라이다(LiDAR) 데이터 전처리 및 8방향 구역 분할
        ranges = np.array(msg.ranges)
        # 센서 노이즈(NaN, Inf) 및 기체 간섭 반경(0.15m 미만) 데이터를 최댓값(12.0)으로 치환
        ranges[np.isinf(ranges) | np.isnan(ranges) | (ranges < 0.15)] = 12.0
        if len(ranges) >= 8:
            chunks = np.array_split(ranges, 8)
            self.latest_scan = [min(float(np.min(c)), 2.0) for c in chunks]

    # 초음파 센서 데이터 갱신 (최대 유효 측정 거리 2.0m 제한)
    def sonar_l_callback(self, msg): self.latest_sonar_l = min(msg.range, 2.0)
    def sonar_c_callback(self, msg): self.latest_sonar_c = min(msg.range, 2.0)
    def sonar_r_callback(self, msg): self.latest_sonar_r = min(msg.range, 2.0)
    
    def gyro_callback(self, msg): 
        # 관성측정장치(IMU) Z축 각속도 데이터 누적 연산 (경로 반전 및 유턴 기동 목적)
        self.latest_gyro_z = msg.data
        current_time = time.time()
        dt = current_time - self.last_gyro_time
        self.last_gyro_time = current_time
        if self.current_goal_state == 1.0:
            self.gyro_z_accum_for_uturn += abs(self.latest_gyro_z * dt)
    
    def odom_callback(self, msg): 
        # 휠 오도메트리 기반 로봇 위치 좌표(X, Y) 갱신
        self.latest_odom_x = msg.pose.pose.position.x
        self.latest_odom_y = msg.pose.pose.position.y
        
    def image_callback(self, msg): 
        # ROS Image 메시지를 OpenCV 호환 포맷(RGB)으로 변환
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def control_loop(self):
        # 비전 센서 프레임이 수신되지 않은 경우 제어 루프 대기
        if self.latest_image is None: return
        
        current_time = time.time()

        # [예외 처리] 물리적 끼임(Stuck) 감지 시 VFH 기반 제자리 회전 탈출 기동
        if self.is_escaping:
            if current_time < self.escape_end_time:
                twist = Twist()
                twist.linear.x = 0.0  
                twist.angular.z = self.escape_angular_z 
                self.cmd_pub.publish(twist)
                return 
            else:
                self.get_logger().info("탈출 기동 완료. 안정화를 위해 2.0초간 끼임 감지 유예 시간(Cooldown)을 적용합니다.")
                self.is_escaping = False
                # 탈출 직후 2초 동안 추가적인 끼임 감지를 무시하여 주행 복귀 유도
                self.escape_cooldown_end = current_time + 2.0 
                
                self.stuck_start_time = current_time
                self.stuck_check_odom_x = self.latest_odom_x
                self.stuck_check_odom_y = self.latest_odom_y

        # [상태 머신 로직] 주행, 목표 지점 도달, 회전, 복귀 완료 상태 전환 관리
        if self.current_goal_state == 0.0 and self.latest_odom_x > 4.5:
            self.get_logger().info("목표 지점(4.5m) 도달 감지. 180도 회전(U-turn) 기동 상태로 전환합니다.")
            self.current_goal_state = 1.0
            self.gyro_z_accum_for_uturn = 0.0 
            
        elif self.current_goal_state == 1.0:
            if self.gyro_z_accum_for_uturn > 150.0:
                self.get_logger().info("방향 전환 임계치 도달. 시작 지점으로의 복귀 기동 상태로 전환합니다.")
                self.current_goal_state = 2.0
                
        elif self.current_goal_state == 2.0 and self.latest_odom_x < 0.3:
             self.get_logger().info("시작 지점 복귀 완료. 자율주행 임무를 종료합니다.")

        # 입력 이미지 전처리: 신경망 입력 규격(224x224) 조정 및 정규화(Normalization)
        resized = cv2.resize(self.latest_image, (224, 224))
        img_normalized = resized.astype(np.float32) / 255.0
        img_normalized = (img_normalized - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        img_input = np.transpose(img_normalized, (2, 0, 1)) 
        img_input = np.expand_dims(img_input, axis=0).astype(np.float32) 
        
        # 다중 센서 데이터 융합을 위한 1차원 텐서(Tensor) 구성
        sensor_array = np.array(
            self.latest_scan + 
            [self.latest_sonar_l, self.latest_sonar_c, self.latest_sonar_r, 
             self.latest_gyro_z, self.current_goal_state],
            dtype=np.float32
        )
        sensor_input = np.expand_dims(sensor_array, axis=0) 

        # AI 모델 추론(Inference) 수행 및 제어 속도(선속도, 각속도) 산출
        outputs = self.session.run(None, {'image_input': img_input, 'sensor_input': sensor_input})
        ai_v, ai_w = float(outputs[0][0][0]), float(outputs[0][0][1])

        # 노이즈 필터링 및 후진 불가(역주행 방지) 강제 제어
        if abs(ai_v) < 0.01: ai_v = 0.0
        if ai_v < 0.0: ai_v = 0.0 
        if abs(ai_w) < 0.01: ai_w = 0.0

        # =====================================================================
        # 위치 변화량 기반 물리적 끼임(Stuck) 감지 알고리즘 적용
        # =====================================================================
        if ai_v > 0.1:
            # 쿨타임(Cooldown) 기간 종료 여부 확인 후 끼임 검사 수행
            if current_time > self.escape_cooldown_end:
                # 3.0초 동안의 누적 위치 변화를 평가
                if current_time - self.stuck_start_time > 3.0:
                    dist_moved = math.hypot(self.latest_odom_x - self.stuck_check_odom_x, 
                                            self.latest_odom_y - self.stuck_check_odom_y)
                    # 설정된 시간 내 이동 거리가 0.02m 미만일 경우 물리적 끼임으로 판정하여 탈출 로직 트리거
                    if dist_moved < 0.02:
                        self.get_logger().error("구동부 물리적 끼임(Stuck) 현상 감지. 위치 복구 기동을 개시합니다.")
                        self.is_escaping = True
                        self.escape_end_time = current_time + 1.2 
                        
                        # 라이다 스캔 데이터 구역 중 개방 공간(최대 거리)을 판별하여 회전 방향 결정
                        max_space_idx = self.latest_scan.index(max(self.latest_scan))
                        if max_space_idx in [1, 2, 3]:
                            self.get_logger().info(f"좌측 가용 공간 확보({max_space_idx}구역). 좌회전 기동 수행.")
                            self.escape_angular_z = 1.5  
                        elif max_space_idx in [5, 6, 7]:
                            self.get_logger().info(f"우측 가용 공간 확보({max_space_idx}구역). 우회전 기동 수행.")
                            self.escape_angular_z = -1.5 
                        else:
                            self.escape_angular_z = 1.5  
                        return
                    else:
                        # 정상 주행 중일 경우 기준 좌표점 지속 갱신
                        self.stuck_start_time = current_time
                        self.stuck_check_odom_x = self.latest_odom_x
                        self.stuck_check_odom_y = self.latest_odom_y
            else:
                # 쿨타임 적용 구간에서는 위치 기준점을 지속적으로 갱신하여 오판 방지
                self.stuck_start_time = current_time
                self.stuck_check_odom_x = self.latest_odom_x
                self.stuck_check_odom_y = self.latest_odom_y
        else:
            # 정지 또는 서행 명령 시 기준점 갱신
            self.stuck_start_time = current_time
            self.stuck_check_odom_x = self.latest_odom_x
            self.stuck_check_odom_y = self.latest_odom_y

        # 전방 충돌 방지(Collision Avoidance) 절대 방어 제어 로직
        front_distance = min(self.latest_scan[0], self.latest_scan[7], self.latest_sonar_c)
        if front_distance < 0.20:
            self.get_logger().warn(f"전방 안전 거리 미달({front_distance:.2f}m). 충돌 방지를 위해 긴급 제동합니다.")
            ai_v, ai_w = 0.0, 0.0
            
        twist = Twist()
        twist.linear.x = ai_v
        twist.angular.z = ai_w
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
