import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import numpy as np

def generate_8figure(a=1.0, T=20, dt=0.02):
    time = np.arange(0, T, dt)

    x = a * np.sin(time)
    y = a * np.sin(time) * np.cos(time)

    dx = a * np.cos(time)
    dy = a * np.cos(2 * time)

    ddx = -a * np.sin(time)
    ddy = -2 * a * np.sin(2 * time)

    v = np.sqrt(dx**2 + dy**2)
    omega = (dx * ddy - dy * ddx) / (v**2 + 1e-8)

    return v, omega

class FigureEightPublisher(Node):
    def __init__(self):
        super().__init__('figure_eight_publisher')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        
        self.index = 0
        self.timer_period = 1e-3  # 1 Hz
        self.v, self.w = generate_8figure(a=1.0, T=20, dt=self.timer_period)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Teleport client
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        # Clear client
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear service...')

        self.reset_position_and_clear()

    def reset_position_and_clear(self):
        # Teleport to origin
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = 0.0
        self.teleport_client.call_async(request)
        # Clear the drawing
        clear_req = Empty.Request()
        self.clear_client.call_async(clear_req)

        self.get_logger().info('Cleared drawing and reset position.')

    def timer_callback(self):
        msg = Twist()
        if self.index < len(self.v):
            msg.linear.x = float(self.v[self.index])
            msg.angular.z = float(self.w[self.index])
            self.index += 1
        else:
            self.reset_position_and_clear()
            self.index = 0
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
