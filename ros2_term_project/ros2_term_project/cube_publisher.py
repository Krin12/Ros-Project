# ROS2용 패키지 임포트
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread, Event


class CubePublisher(Node):  # Node 상속
    # 발행률 정적 변수 정의
    PUB_RATE = 10.0

    def __init__(self):
        # Node의 node_name 초기화
        super().__init__('cube_publisher')
        # Publisher 객체 생성
        self.publisher_ = self.create_publisher(Twist, 'cube_cmd_vel', 1)
        timer_period = 1 / CubePublisher.PUB_RATE  # seconds
        # Timer 객체 생성
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 선속도 인스턴스 변수 정의
        self.current_linear_x = -1.0
        # 방향 전환을 위한 별도의 스레드
        self.thread = Thread(target=self.turn_cube, daemon=True)
        self.thread.start()

    # Timer 콜백 메서드 정의
    def timer_callback(self):
        msg = Twist()   # Twist 객체 생성
        msg.linear.x = float(self.current_linear_x) # 선속도 할당
        self.get_logger().info('linear.x = %.3f' % msg.linear.x)    # info
        self.publisher_.publish(msg)    # 토픽 발행

    # 방향 전환 메소드
    def turn_cube(self):
        while True:
            Event().wait(14.0)  # 14초마다 실행
            self.current_linear_x = -self.current_linear_x
            self.get_logger().info('turn success!!@')


def main(args=None):
    # 현재의 ROS2 컨텍스트에 대한 통신 초기화
    rclpy.init(args=args)
    # CubePublisher 객체 생성
    cube_publisher = CubePublisher()

    try:
        rclpy.spin(cube_publisher)
    except KeyboardInterrupt:
        # Ctrl-C 누를 때 메시지 출력
        cube_publisher.get_logger().info('user-terminated by pressing Ctrl-C...')

    # rclpy.shutdown()에서 자동으로 처리하기 때문에 반드시 사용할 필요는 없음
    cube_publisher.destroy_node()
    # ROS2 컨텍스트 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()