import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('circle_follower')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.01  # seconds (1 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Circle follower started")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0   # Forward speed [m/s]
        msg.angular.z = 1.0  # Yaw rotation [rad/s]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

