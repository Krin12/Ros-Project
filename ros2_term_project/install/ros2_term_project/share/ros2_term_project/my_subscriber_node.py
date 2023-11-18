import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelState

def callback(msg, set_entity_state):
    print('Received: Linear=%f, Angular=%f' % (msg.linear.x, msg.angular.z))

    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    model_state = ModelState()
    model_state.model_name = 'prius_hybrid'
    model_state.twist.linear.x = linear_velocity
    model_state.twist.angular.z = angular_velocity

    set_entity_state(model_state)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('my_subscriber_node')
    subscription = node.create_subscription(Twist, '/prius_hybrid/cmd_vel', callback, 10)

    set_entity_state = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    set_entity_state.wait_for_service()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

