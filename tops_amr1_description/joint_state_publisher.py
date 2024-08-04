import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.last_time = self.get_clock().now()

        # Initialize joint state
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_rear_wheel_1', 'base_to_rear_wheel_2']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = []

    def publish_joint_states(self):
        current_time = self.get_clock().now()

        # Update joint positions (dummy data for example, replace with actual data)
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.joint_state.position[0] += 0.1 * delta_time  # Simulate wheel rotation
        self.joint_state.position[1] += 0.1 * delta_time

        self.joint_state.header.stamp = current_time.to_msg()
        self.joint_pub.publish(self.joint_state)

        # Broadcast the transform for rear_wheel_link_1
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rear_wheel_link_1'
        t.transform.translation.x = -0.264  # from URDF origin xyz of base_to_rear_wheel_1
        t.transform.translation.y = 0.21
        t.transform.translation.z = 0.0975
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 1.0  # z = 1.0 corresponds to 180 degrees (pi) rotation
        t.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(t)

        # Broadcast the transform for rear_wheel_link_2
        t.child_frame_id = 'rear_wheel_link_2'
        t.transform.translation.x = 0.264  # from URDF origin xyz of base_to_rear_wheel_2
        t.transform.translation.y = 0.21
        t.transform.translation.z = 0.0975
        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

