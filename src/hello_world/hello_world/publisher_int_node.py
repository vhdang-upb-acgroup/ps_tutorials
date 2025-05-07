import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8



class GradePublisher(Node):

    def __init__(self):
        super().__init__('RAT_publisher')
        self.publisher_ = self.create_publisher(Int8, 'grade', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.publish_number)

    def publish_number(self):
        msg = Int8()
        msg.data = 8
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    grade_node = GradePublisher()
    rclpy.spin(grade_node)

    grade_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
