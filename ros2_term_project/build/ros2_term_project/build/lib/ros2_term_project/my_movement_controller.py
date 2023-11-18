import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MovementControllerNode:

    def __init__(self):
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(String, 'control_commands', self.control_callback, 10)
        self.get_logger().info("움직임 제어 노드 실행 중.")

    def control_callback(self, msg):
        # 명령은 "linear_velocity,angular_velocity" 형식으로 가정합니다.
        commands = msg.data.split(',')
        linear_velocity = float(commands[0])
        angular_velocity = float(commands[1])

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity

        self.publisher.publish(twist_msg)
        self.get_logger().info(f"Twist 메시지 발행: {twist_msg}")

def main(args=None):
    rclpy.init(args=args)

    movement_controller_node = MovementControllerNode()

    rclpy.spin(movement_controller_node)

    movement_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

