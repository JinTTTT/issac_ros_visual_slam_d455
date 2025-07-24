import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        transforms = []

        # camera_link -> camera_gyro_optical_frame
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'camera_link'
        t1.child_frame_id = 'camera_gyro_frame'
        t1.transform.translation.x = -0.016019999980926514
        t1.transform.translation.y = -0.030220000073313713
        t1.transform.translation.z = 0.007400000002235174
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'camera_gyro_frame'
        t2.child_frame_id = 'camera_gyro_optical_frame'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = -0.5
        t2.transform.rotation.y = 0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = 0.5
        transforms.append(t2)

        # camera_link -> camera_infra1_frame
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'camera_link'
        t3.child_frame_id = 'camera_infra1_frame'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        transforms.append(t3)

        # camera_link -> camera_infra2_frame
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'camera_link'
        t4.child_frame_id = 'camera_infra2_frame'
        t4.transform.translation.x = 0.0
        t4.transform.translation.y = -0.09518329054117203
        t4.transform.translation.z = 0.0
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0
        transforms.append(t4)

        # camera_infra1_frame -> camera_infra1_optical_frame
        t5 = TransformStamped()
        t5.header.stamp = self.get_clock().now().to_msg()
        t5.header.frame_id = 'camera_infra1_frame'
        t5.child_frame_id = 'camera_infra1_optical_frame'
        t5.transform.translation.x = 0.0
        t5.transform.translation.y = 0.0
        t5.transform.translation.z = 0.0
        t5.transform.rotation.x = -0.5
        t5.transform.rotation.y = 0.5
        t5.transform.rotation.z = -0.5
        t5.transform.rotation.w = 0.5
        transforms.append(t5)

        # camera_infra2_frame -> camera_infra2_optical_frame
        t6 = TransformStamped()
        t6.header.stamp = self.get_clock().now().to_msg()
        t6.header.frame_id = 'camera_infra2_frame'
        t6.child_frame_id = 'camera_infra2_optical_frame'
        t6.transform.translation.x = 0.0
        t6.transform.translation.y = 0.0
        t6.transform.translation.z = 0.0
        t6.transform.rotation.x = -0.5
        t6.transform.rotation.y = 0.5
        t6.transform.rotation.z = -0.5
        t6.transform.rotation.w = 0.5
        transforms.append(t6)

        # publish all static transforms
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info('published static transforms')

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 