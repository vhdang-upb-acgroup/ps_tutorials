import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import numpy as np
import math


def generate_8figure_trajectory(a=2.0, T=20.0, dt=0.1):
    t = np.arange(0, T, dt)

    # Scale time to complete one loop in T seconds
    omega = 2 * np.pi / T  # This ensures one full cycle in T seconds

    x = a * np.sin(omega * t)
    y = a * np.sin(omega * t) * np.cos(omega * t)

    return list(zip(x + 5.5, y + 5.5))

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_trajectory_follower')

        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)

        # Teleport client
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        # Clear client
        self.clear_client = self.create_client(Empty, '/clear')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for clear service...')

        # Parameters
        self.index = 0
        self.sample_time = 1e-3 # 1ms
        self.goal_tolerance = 0.2  # How close to goal point before advancing

        # PID gains (just P here)
        self.Kp_linear = 1.5
        self.Kp_angular = 6.0

        self.current_pose = None
        self.timer = self.create_timer(self.sample_time, self.control_loop)
        # Generate reference trajectory
        self.trajectory = generate_8figure_trajectory(a=2, T=20, dt=self.sample_time)

        self.reset_position_and_clear()

    def reset_position_and_clear(self):
        # Clear the drawing
        clear_req = Empty.Request()
        self.clear_client.call_async(clear_req)

        # Teleport to origin
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = 0.0
        self.teleport_client.call_async(request)

        self.get_logger().info('Cleared drawing and reset position.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None or self.index >= len(self.trajectory):
            self.index = 0
            self.reset_position_and_clear()
            self.trajectory = generate_8figure_trajectory(a=2, T=20, dt=self.sample_time)
            return

        # Get current pose
        x = self.current_pose.x
        y = self.current_pose.y
        theta = self.current_pose.theta

        # Target point
        target_x, target_y = self.trajectory[self.index]

        # Compute errors
        dx = target_x - x
        dy = target_y - y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_theta - theta)

        # Simple P controller
        linear = self.Kp_linear * distance_error
        angular = self.Kp_angular * angle_error

        # Limit speeds
        linear = min(linear, 5.0)
        angular = max(min(angular, 8.0), -8.0)

        # Publish
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.index += 1
        
        self.publisher_.publish(twist)

        # # Advance to next point if close
        # if distance_error < self.goal_tolerance:
        #     self.index += 1
        #     if self.index >= len(self.trajectory):
        #         self.get_logger().info("Trajectory completed.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
