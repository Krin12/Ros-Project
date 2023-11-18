import rclpy
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('my_controller_node')
    publisher = node.create_publisher(Twist, '/prius_hybrid/cmd_vel', 10)

    msg = Twist()
    msg.linear.x = 1.0  # 선속도 (앞으로 전진)
    msg.angular.z = 0.5  # 각속도 (시계방향 회전)

    try:
        while rclpy.ok():
            publisher.publish(msg)
            node.get_logger().info('Publishing: Linear=%f, Angular=%f' % (msg.linear.x, msg.angular.z))
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
