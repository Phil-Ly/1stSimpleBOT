import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('topic_name', 'chatter')

        freq = self.get_parameter('publish_frequency').value
        topic = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.i = 0
        self.get_logger().info(f"Publisher started: topic={topic}, freq={freq}")

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()