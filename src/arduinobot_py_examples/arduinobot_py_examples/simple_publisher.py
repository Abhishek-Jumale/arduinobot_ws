import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Counter
        self.counter_ = 0

        # Frequency in Hz
        self.frequency_ = 1.0  # 1 Hz = every 1 second
        self.get_logger().info("Pblishing at %.1f Hz" % self.frequency_)

        # Timer (period = 1/frequency)
        self.timer_ = self.create_timer(1.0 / self.frequency_, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
