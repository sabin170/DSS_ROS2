#!/usr/bin/env python3

import sys
import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from PyQt5 import QtWidgets, uic
from ament_index_python.packages import get_package_share_directory

class SimpleSteeringGUI(QtWidgets.QDialog):
    def __init__(self):
        # Qt 초기화
        super().__init__()
        
        # ROS2 노드 초기화 (별도로)
        rclpy.init()
        self.node = rclpy.create_node('simple_steering_gui')
        
        # UI 파일 로드
        try:
            package_dir = get_package_share_directory('dss_imu_steering_control')
            ui_file = os.path.join(package_dir, 'ui', 'steering_control.ui')
            uic.loadUi(ui_file, self)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to load UI file from package: {e}")
            # 폴백: 직접 경로 사용
            try:
                ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui', 'steering_control.ui')
                uic.loadUi(ui_file, self)
                self.node.get_logger().info(f"Loaded UI file from fallback path: {ui_file}")
            except Exception as fallback_e:
                self.node.get_logger().error(f"Failed to load UI file from fallback path: {fallback_e}")
                raise fallback_e
        
        # 윈도우 설정
        self.setWindowTitle("Simple Steering Control")
        self.setFixedSize(351, 224)
        
        # 상태 변수
        self.control_enabled = False
        self.current_speed = 0.0
        
        # ROS2 Publishers (토픽만 발행)
        self.target_angle_pub = self.node.create_publisher(Float32, '/steering/target_angle', 1)
        self.enable_pub = self.node.create_publisher(Bool, '/steering/enable', 1)
        self.throttle_pub = self.node.create_publisher(Float32, '/car/throttle', 1)
        
        # UI 신호 연결
        self.setup_ui_connections()
        
        self.node.get_logger().info("🎮 Simple Steering GUI Started - Publishing topics only")
    
    def setup_ui_connections(self):
        """UI 신호 연결 설정"""
        # 버튼 클릭 이벤트 연결
        self.pushButton_Enable_Heading_Control.clicked.connect(self.toggle_control)
        self.pushButton_Target_Angle.clicked.connect(self.set_target_angle)
        
        # Enter 키로 목표 각도 설정
        self.lineEdit_Target_Angle.returnPressed.connect(self.set_target_angle)
        
        # 슬라이더 이벤트 연결
        self.horizontalSlider_Car_Speed.valueChanged.connect(self.speed_changed)
        
        # 초기값 설정
        self.lineEdit_Target_Angle.setText("0.0")
        self.horizontalSlider_Car_Speed.setRange(0, 6)  # 0-6 (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30)
        self.horizontalSlider_Car_Speed.setValue(0)  # 초기값 0
        self.update_button_state()
    
    def toggle_control(self):
        """제어 활성화/비활성화 토글 - 토픽 발행"""
        self.control_enabled = not self.control_enabled
        
        # ROS2 토픽 발행
        msg = Bool()
        msg.data = self.control_enabled
        self.enable_pub.publish(msg)
        
        self.update_button_state()
        
        status = "ENABLED" if self.control_enabled else "DISABLED"
        self.node.get_logger().info(f"📡 Publishing enable: {self.control_enabled} - Control {status}")
    
    def set_target_angle(self):
        """목표 각도 설정 - 토픽 발행"""
        try:
            # 입력값을 도(degree)에서 라디안(radian)으로 변환
            angle_deg = float(self.lineEdit_Target_Angle.text())
            angle_rad = math.radians(angle_deg)
            
            # 각도 정규화 (-π ~ π)
            angle_rad = self.normalize_angle(angle_rad)
            
            # ROS2 토픽 발행
            msg = Float32()
            msg.data = angle_rad
            self.target_angle_pub.publish(msg)
            
            # 입력 필드 업데이트 (정규화된 값으로)
            angle_deg_normalized = math.degrees(angle_rad)
            self.lineEdit_Target_Angle.setText(f"{angle_deg_normalized:.1f}")
            
            self.node.get_logger().info(f"📡 Publishing target angle: {angle_deg_normalized:.1f}° ({angle_rad:.3f} rad)")
            
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "입력 오류", "유효한 각도를 입력하세요 (예: 90, -45)")
            self.lineEdit_Target_Angle.setText("0.0")
    
    def update_button_state(self):
        """버튼 상태 업데이트"""
        if self.control_enabled:
            self.pushButton_Enable_Heading_Control.setText("Heading Control Disable")
            self.pushButton_Enable_Heading_Control.setStyleSheet("background-color: #ff6b6b; color: white;")
        else:
            self.pushButton_Enable_Heading_Control.setText("Heading Control Enable")
            self.pushButton_Enable_Heading_Control.setStyleSheet("background-color: #51cf66; color: white;")
    
    def speed_changed(self, value):
        """속도 슬라이더 값 변경 - 토픽 발행"""
        # 슬라이더 값(0-6)을 스로틀 값(0.0-0.3)으로 변환 (0.05 단위)
        throttle_value = value * 0.05
        self.current_speed = throttle_value
        
        # ROS2 토픽 발행
        msg = Float32()
        msg.data = throttle_value
        self.throttle_pub.publish(msg)
        
        self.node.get_logger().info(f"📡 Publishing throttle: {throttle_value:.2f} (slider: {value})")
    
    def normalize_angle(self, angle):
        """각도 정규화 (-π ~ π)"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def closeEvent(self, event):
        """윈도우 닫기 이벤트"""
        # 제어 비활성화 및 속도 0으로 설정
        if self.control_enabled:
            msg = Bool()
            msg.data = False
            self.enable_pub.publish(msg)
            self.node.get_logger().info("📡 Publishing disable on close")
        
        # 스로틀을 0으로 설정
        if self.current_speed > 0:
            throttle_msg = Float32()
            throttle_msg.data = 0.0
            self.throttle_pub.publish(throttle_msg)
            self.node.get_logger().info("📡 Publishing zero throttle on close")
        
        self.node.get_logger().info("🎮 Simple GUI Closing...")
        # ROS2 정리
        self.node.destroy_node()
        event.accept()

def main():
    # ROS2는 GUI 클래스 내부에서 초기화
    app = QtWidgets.QApplication(sys.argv)
    
    try:
        window = SimpleSteeringGUI()
        window.show()
        
        # Qt 이벤트 루프 실행
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Error starting GUI: {e}")
        return 1
    
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
