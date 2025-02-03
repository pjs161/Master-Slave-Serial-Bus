import rclpy  # ROS2 라이브러리 불러오기
from rclpy.node import Node  # ROS2 노드 클래스 불러오기
from dynamixel_sdk import *  # Dynamixel SDK 불러오기
from std_msgs.msg import Float64MultiArray  # 메시지 타입 불러오기

class SlaveController(Node):
    def __init__(self):
        super().__init__('slave_controller')  # 노드 이름 설정

        # Dynamixel 설정
        self.port_handler = PortHandler('/dev/ttyUSB0')  # U2D2 포트 설정
        self.packet_handler = PacketHandler(2.0)        # Protocol 2.0 설정
        self.motor_ids = [8, 9, 10, 11, 12]  # 슬레이브 모터 ID 리스트 설정

        # 포트 열기
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open port')  # 포트 열기 실패 시 에러 로그 출력
            exit()
        if not self.port_handler.setBaudRate(57600):
            self.get_logger().error('Failed to set baud rate')  # 보드레이트 설정 실패 시 에러 로그 출력
            exit()

        # 모터 초기화
        self.init_motors()

        # ROS2 서브스크라이버 설정
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'master_positions',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning (미사용 변수 경고 방지)

    def init_motors(self):
        for motor_id in self.motor_ids:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, 64, 1)  # Torque Enable 설정
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(f"Failed to enable torque for Motor ID {motor_id}")  # 토크 설정 실패 시 에러 로그 출력
            else:
                self.get_logger().info(f"Torque enabled for Motor ID {motor_id}")  # 토크 설정 성공 시 로그 출력

    def listener_callback(self, msg):
        master_positions = msg.data
        self.get_logger().info(f"Received Motor Positions from Master: {master_positions}")  # 마스터로부터 받은 위치 데이터 로그 출력

        if not master_positions:
            self.get_logger().error("No data received from Master!")  # 데이터 수신 실패 시 에러 로그 출력
            return

        # 슬레이브 모터 ID와 마스터 위치 데이터 매핑
        slave_positions = {
            8: master_positions[0],  # 마스터 ID 2 → 슬레이브 ID 8
            9: master_positions[1],  # 마스터 ID 3 → 슬레이브 ID 9
            10: master_positions[2],  # 마스터 ID 4 → 슬레이브 ID 10
            11: master_positions[3],  # 마스터 ID 5 → 슬레이브 ID 11
            12: master_positions[4],  # 마스터 ID 6 → 슬레이브 ID 12
        }
        self.set_positions(slave_positions)  # 슬레이브 모터 위치 설정

    def set_positions(self, positions):
        for motor_id, position in positions.items():
            position = max(0, min(4095, int(position)))  # 위치 값 범위 제한
            result, error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, 116, position)  # 위치 설정
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(f"Failed to set position for Motor ID {motor_id}: {position}")  # 위치 설정 실패 시 에러 로그 출력
            else:
                self.get_logger().info(f"Set position for Motor ID {motor_id}: {position}")  # 위치 설정 성공 시 로그 출력

def main():
    rclpy.init()  # ROS2 초기화
    node = SlaveController()  # SlaveController 노드 생성
    rclpy.spin(node)  # 노드 실행
    node.destroy_node()  # 노드 삭제
    rclpy.shutdown()  # ROS2 종료

