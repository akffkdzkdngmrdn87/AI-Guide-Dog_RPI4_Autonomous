#!/usr/bin/env python3
# =====================================================================
# Slamtec SLLiDAR C1 공식 런치(Launch) 스크립트 최적화 버전
# 라이다 센서 하드웨어 구동 파라미터(Parameter) 선언 및 ROS 2 노드 실행 정의
# =====================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 센서 하드웨어 연결 및 데이터 취득을 위한 기본 매개변수 변수 설정
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='라이다 장치 통신 채널 유형 지정 (serial 또는 udp)'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='라이다가 매핑된 USB 포트 절대 경로 지정 (기본값: /dev/ttyUSB0)'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='USB 직렬 포트 통신 보드레이트(Baudrate) 속도 지정 (C1 모델 권장: 460800)'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='ROS 2 TF(Transform) 트리에 등록할 라이다의 기준 프레임 ID 지정'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='기구학적 장착 방향(상하 반전)에 따른 스캔 데이터 역상(Invert) 출력 여부 지정'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='스캔 데이터의 각도 보상(Angle Compensation) 및 보간 기능 활성화 여부 지정'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='라이다 샘플링 주파수 및 스캔 밀도 모드 지정 (기본값: Standard)'),

        # sllidar_ros2 패키지의 핵심 실행 파일(sllidar_node) 호출 및 런타임 파라미터 매핑
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen'),
    ])
