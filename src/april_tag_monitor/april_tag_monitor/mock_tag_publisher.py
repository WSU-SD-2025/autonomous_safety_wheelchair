import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MockTagPublisher(Node):
    def __init__(self):
        super().__init__('mock_tag_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/front_rgbd/tag_detections', 10)
        self.timer = self.create_timer(1.0, self.publish_mock_tag)

    def publish_mock_tag(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # use frame id consistent with your system
        msg.pose.position.x = 1.0  # within your threshold of 1.8m
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.5
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info('Published mock PoseStamped tag detection.')

def main(args=None):
    rclpy.init(args=args)
    node = MockTagPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
