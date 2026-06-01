#!/usr/bin/env python3
# =====================================================================
# ROS 2 - 하위 제어기(Arduino) 간 직렬 통신 브리지(Serial Bridge) 노드
# 모터 제어 명령(cmd_vel) 문자 변환 및 센서 데이터(IMU, 초음파) 발행
# =====================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Range 
import serial

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        
        # 구독 및 발행 토픽 선언
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.imu_pub = self.create_publisher(Float32, 'gyro_z', 10)
        self.sonar_l_pub = self.create_publisher(Range, 'sonar_left', 10)
        self.sonar_c_pub = self.create_publisher(Range, 'sonar_center', 10)
        self.sonar_r_pub = self.create_publisher(Range, 'sonar_right', 10)
        
        # 하위 제어기(MCU) 직렬 포트 연결 초기화
        try: 
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("하위 제어기(Arduino)와의 직렬 통신(Serial)이 성공적으로 연결되었습니다.")
        except Exception as e: 
            self.get_logger().error(f"하위 제어기 직렬 통신 연결 실패 (포트 및 권한 확인 필요): {e}")
            
        # 50Hz(0.02초) 주기로 시리얼 데이터 수신 처리 타이머 할당
        self.timer = self.create_timer(0.02, self.read_serial) 

    def cmd_vel_callback(self, msg):
        # 수신된 cmd_vel(선속도/각속도) 데이터를 아두이노 전송용 단일 문자 명령어로 매핑
        cmd = 's' # 기본 상태: 정지(Stop)
        
        if msg.linear.x > 0.001 and msg.angular.z > 0.001: 
            cmd = 'q' # 좌측 전방 대각선 기동
        elif msg.linear.x > 0.001 and msg.angular.z < -0.001: 
            cmd = 'e' # 우측 전방 대각선 기동
            
        elif msg.linear.x > 0.001: cmd = 'f' # 전진(Forward)
        elif msg.linear.x < -0.001: cmd = 'b' # 후진(Backward)
        elif msg.angular.z > 0.001: cmd = 'l' # 제자리 좌회전(Left turn)
        elif msg.angular.z < -0.001: cmd = 'r' # 제자리 우회전(Right turn)

        # 하위 제어기 통신 프로토콜('C,명령어\n')에 맞추어 인코딩 후 전송
        send_data = f"C,{cmd}\n"
        if hasattr(self, 'ser'): self.ser.write(send_data.encode()) 

    def read_serial(self):
        # 하위 제어기로부터 수신된 센서 문자열 파싱 및 ROS 2 메시지 규격 변환
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                # 관성측정장치(IMU) Z축 각속도 데이터 파싱
                if line.startswith("I,"): 
                    msg = Float32()
                    msg.data = float(line.split(',')[1])
                    self.imu_pub.publish(msg) 
                    
                # 3채널 초음파 센서 배열 데이터 파싱
                elif line.startswith("U,"):
                    parts = line.split(',')
                    if len(parts) == 4:
                        dl, dc, dr = float(parts[1]), float(parts[2]), float(parts[3])
                        self.sonar_l_pub.publish(self.create_range_msg(dl))
                        self.sonar_c_pub.publish(self.create_range_msg(dc))
                        self.sonar_r_pub.publish(self.create_range_msg(dr))
            except: pass

    def create_range_msg(self, dist_cm):
        # 초음파 측정 거리(cm)를 ROS 2 표준 Range 메시지 규격(m)으로 정규화
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' 
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26 # 센서 빔 확산각 설정 (약 15도)
        msg.min_range = 0.02     # 최소 유효 측정 거리 설정 (0.02m)
        msg.max_range = 2.0      # 최대 유효 측정 거리 한계 설정 (2.0m)
        msg.range = dist_cm / 100.0 if dist_cm < 200 else 2.0 
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
