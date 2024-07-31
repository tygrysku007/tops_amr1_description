import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class EncoderSubscriber(Node):

    def __init__(self):
        super().__init__('encoder_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        left_wheel_position = msg.pose.pose.position.x
        right_wheel_position = msg.pose.pose.position.y
        self.get_logger().info('Left Wheel Position: %f, Right Wheel Position: %f' % (left_wheel_position, right_wheel_position))

def main(args=None):
    rclpy.init(args=args)
    encoder_subscriber = EncoderSubscriber()
    rclpy.spin(encoder_subscriber)
    encoder_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

