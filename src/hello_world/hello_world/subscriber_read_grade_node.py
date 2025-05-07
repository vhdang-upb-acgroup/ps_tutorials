import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8


class ReadGrade(Node):
    def __init__(self):
        super().__init__('read_grade_subscriber')

        self.subcriber_ = self.create_subscription(
            Int8,
            'grade',
            self.read_grade,
            10
        )
        self.subcriber_  # prevent unused variable warning
    def read_grade(self,msg):
        print(f'I got: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    read_node = ReadGrade()
    rclpy.spin(read_node)

    read_node.destroy_node(read_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
