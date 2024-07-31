import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        orientation_data = msg.orientation
        angular_velocity_data = msg.angular_velocity
        linear_acceleration_data = msg.linear_acceleration

        # Example: Print or process the IMU data
        self.get_logger().info('Orientation: x=%f, y=%f, z=%f, w=%f' %
                               (orientation_data.x, orientation_data.y,
                                orientation_data.z, orientation_data.w))
        self.get_logger().info('Angular Velocity: x=%f, y=%f, z=%f' %
                               (angular_velocity_data.x, angular_velocity_data.y, angular_velocity_data.z))
        self.get_logger().info('Linear Acceleration: x=%f, y=%f, z=%f' %
                               (linear_acceleration_data.x, linear_acceleration_data.y, linear_acceleration_data.z))

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

