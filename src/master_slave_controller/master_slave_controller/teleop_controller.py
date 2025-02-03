import rclpy  # ROS2 라이브러리 불러오기
from rclpy.node import Node  # ROS2 노드 클래스 불러오기
from dynamixel_sdk import *  # Dynamixel SDK 불러오기
from std_msgs.msg import Float64MultiArray  # 메시지 타입 불러오기

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')  # 노드 이름 설정

        # Dynamixel 설정
        self.port_handler = PortHandler('/dev/ttyUSB0')  # U2D2 포트 설정
        self.packet_handler = PacketHandler(2.0)        # Protocol 2.0 설정
        self.motor_ids = [2, 3, 4, 5, 6]  # 마스터 모터 ID 리스트 설정

        # 포트 열기
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open port')  # 포트 열기 실패 시 에러 로그 출력
            exit()
        if not self.port_handler.setBaudRate(57600):
            self.get_logger().error('Failed to set baud rate')  # 보드레이트 설정 실패 시 에러 로그 출력
            exit()

        # Torque Disable
        self.disable_torque()  # Torque 비활성화 메서드 호출

        # ROS2 퍼블리셔 설정
        self.publisher = self.create_publisher(Float64MultiArray, 'master_positions', 10)  # 퍼블리셔 생성

        # ROS2 타이머 설정 (0.1초 간격으로 위치 퍼블리싱)
        self.timer = self.create_timer(0.1, self.publish_positions)  # 타이머 생성 및 콜백 설정

    def disable_torque(self):
        """
        Master 모터의 Torque를 비활성화하여 손으로 움직일 수 있도록 설정.
        """
        for motor_id in self.motor_ids:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 64, 0)  # Torque Disable
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(f"Failed to disable torque for Motor ID {motor_id}")  # Torque 비활성화 실패 시 에러 로그 출력
            else:
                self.get_logger().info(f"Torque disabled for Motor ID {motor_id}")  # Torque 비활성화 성공 시 로그 출력

    def read_positions(self):
        """
        현재 모터 위치를 읽어 반환합니다.
        """
        positions = []
        for motor_id in self.motor_ids:
            position, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, 132)  # 현재 위치 읽기
            positions.append(float(position))  # float 타입으로 변환하여 리스트에 추가
        return positions

    def publish_positions(self):
        """
        현재 모터 위치를 퍼블리시합니다.
        """
        positions = self.read_positions()  # 현재 위치 읽기
        msg = Float64MultiArray()  # 메시지 객체 생성
        msg.data = positions  # 데이터 설정
        self.publisher.publish(msg)  # 메시지 퍼블리시
        self.get_logger().info(f"Published Motor Positions: {positions}")  # 퍼블리시된 위치 로그 출력

def main():
    rclpy.init()  # ROS2 초기화
    node = MasterController()  # MasterController 노드 생성
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 노드 삭제
    rclpy.shutdown()  # ROS2 종료

