#!/usr/bin/env python3
# =====================================================================
# 센서 퓨전 기반 오도메트리(Odometry) 연산 및 변환(TF) 발행 노드
# 휠 엔코더 펄스(Pulse) 및 IMU 자이로 데이터의 기구학적(Kinematics) 결합 모델
# =====================================================================

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
        
        # 싱글 보드 컴퓨터(SBC) 하드웨어 인터럽트 연결 GPIO 핀 정의
        self.ENC_FL = 17; self.ENC_FR = 27; self.ENC_RL = 22; self.ENC_RR = 23
        
        # 기구학적 제원(Wheel Kinematics) 정의
        WHEEL_DIAMETER = 0.066 # 바퀴 직경 66mm
        TICKS_PER_REV = 20     # 1회전 당 엔코더 펄스 수
        self.METERS_PER_TICK = (math.pi * WHEEL_DIAMETER) / TICKS_PER_REV # 1펄스 당 선형 이동 거리 산출
        
        # 스키드 스티어링(Skid-Steering) 주행 방식의 휠 슬립(Slip) 현상 수학적 보정 계수
        self.SLIP_FACTOR = 0.95 
        
        # 좌표 및 방향 추적을 위한 상태 변수 초기화
        self.ticks = 0
        self.is_moving_forward = False 
        self.x = 0.0; self.y = 0.0; self.th = 0.0; self.vth = 0.0 
        self.last_time = self.get_clock().now()
        
        # GPIO 핀 모드 할당 및 인터럽트(Event Detect) 콜백 설정
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.ENC_FL, self.ENC_FR, self.ENC_RL, self.ENC_RR], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.ENC_FL, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_FR, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_RL, GPIO.BOTH, callback=self.encoder_cb)
        GPIO.add_event_detect(self.ENC_RR, GPIO.BOTH, callback=self.encoder_cb)

        # ROS 2 토픽 구독 및 발행 선언
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Float32, 'gyro_z', self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) 
        
        # 20Hz(0.05초) 주기로 오도메트리 연산 및 발행 수행
        self.timer = self.create_timer(0.05, self.publish_odom)
        self.get_logger().info("센서 퓨전 기반 오도메트리(Odometry) 발행 노드가 정상적으로 실행되었습니다.")

    def encoder_cb(self, channel):
        # 전진 기동 상태일 때만 엔코더 펄스를 누적하여 후진 시 이동량 상쇄 오차 방지
        if self.is_moving_forward: self.ticks += 1 

    def cmd_vel_callback(self, msg):
        # 제어 명령(cmd_vel)의 선속도 벡터를 판별하여 전진 플래그 상태 갱신
        self.is_moving_forward = True if msg.linear.x > 0.001 else False

    def imu_callback(self, msg):
        # IMU 노이즈 데드존(0.1도/s) 필터링 후 각속도를 라디안 단위(Radian/sec)로 변환 적용
        self.vth = msg.data * (math.pi / 180.0) if abs(msg.data) > 0.1 else 0.0 

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        
        # 4개 구동륜의 평균 회전량 산출 및 슬립 보정 계수 적용 (선형 이동 거리 도출)
        delta_dist = ((self.ticks / 4.0) * self.METERS_PER_TICK) * self.SLIP_FACTOR
        self.ticks = 0 
        vx = delta_dist / dt if dt > 0 else 0.0 
        
        # 누적 위치(X, Y) 및 누적 방위각(Theta) 벡터 연산
        delta_x = delta_dist * math.cos(self.th)
        delta_y = delta_dist * math.sin(self.th)
        delta_th = self.vth * dt
        
        self.x += delta_x; self.y += delta_y; self.th += delta_th
        
        # 기준 좌표계 간 변환 정보(Transform) 설정 (odom 프레임 -> base_link 프레임)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg(); t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'         
        t.transform.translation.x = self.x; t.transform.translation.y = self.y; t.transform.translation.z = 0.0
        
        # 오일러 각도(Euler Angle)를 3차원 공간 계산 안정화를 위한 쿼터니언(Quaternion)으로 변환
        q = self.euler_to_quaternion(0, 0, self.th) 
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]; t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t) 
        
        # 위치 및 속도 정보를 포함한 오도메트리 메시지 패키징
        odom = Odometry()
        odom.header.stamp = current_time.to_msg(); odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]; odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx; odom.twist.twist.angular.z = self.vth
        
        # 내비게이션 스택(Nav2) 호환성을 위한 공분산(Covariance) 행렬 할당 (비활성 축에 큰 분산값 부여)
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
        # 회전 변환을 위한 오일러-쿼터니언 수학적 변환 함수
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
