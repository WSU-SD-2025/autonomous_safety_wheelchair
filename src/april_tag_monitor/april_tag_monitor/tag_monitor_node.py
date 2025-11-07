import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import math

class TagMonitorNode(Node):
    def __init__(self):
        super().__init__('tag_monitor_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('front_camera_topic', '/front_rgbd/tag_detections'),
                ('rear_camera_topic', '/rear_rgbd/tag_detections'),
                ('tag_distance_threshold_m', 1.8),
                ('no_tag_timeout_sec', 30.0),
                ('tag_detection_frame', 'base_link'),
                ('enable_auto_mode_switching', True),
                ('april_destination_topic', '/april_destination'),
                ('person_back_topic', '/person_back'),
                ('no_tag_topic', '/no_tag_detected'),
            ],
        )

        # Load parameters
        self.front_camera_topic = self.get_parameter('front_camera_topic').value
        self.rear_camera_topic = self.get_parameter('rear_camera_topic').value
        self.tag_distance_threshold = self.get_parameter('tag_distance_threshold_m').value
        self.no_tag_timeout = self.get_parameter('no_tag_timeout_sec').value
        self.tag_frame = self.get_parameter('tag_detection_frame').value
        self.auto_mode_switching = self.get_parameter('enable_auto_mode_switching').value

        self.april_destination_topic = self.get_parameter('april_destination_topic').value
        self.person_back_topic = self.get_parameter('person_back_topic').value
        self.no_tag_topic = self.get_parameter('no_tag_topic').value

        # State
        self.last_detection_time = None
        self.last_detection_source = None

        # Publishers
        self.destination_pub = self.create_publisher(PoseStamped, self.april_destination_topic, 10)
        self.person_back_pub = self.create_publisher(Bool, self.person_back_topic, 10)
        self.no_tag_pub = self.create_publisher(Bool, self.no_tag_topic, 10)

        # Subscriptions (now for PoseStamped messages)
        self.create_subscription(
            PoseStamped,
            self.front_camera_topic,
            self.front_tag_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            self.rear_camera_topic,
            self.rear_tag_callback,
            10
        )

        # Timer for checking tag detection timeout
        self.create_timer(2.0, self.tag_detection_timer)

        self.get_logger().info('TagMonitorNode started.')

    def front_tag_callback(self, msg: PoseStamped):
        self.process_tag_detection(msg, source='front')

    def rear_tag_callback(self, msg: PoseStamped):
        if self.last_detection_time is None:
            return

        time_diff_sec = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_diff_sec > self.no_tag_timeout:
            return

        self.process_tag_detection(msg, source='rear')

    def process_tag_detection(self, pose_msg: PoseStamped, source='front'):
        distance = self.calculate_tag_distance(pose_msg.pose)

        if distance <= self.tag_distance_threshold:
            self.publish_person_detected(True)
            self.publish_april_destination(pose_msg.pose)

            self.last_detection_time = self.get_clock().now()
            self.last_detection_source = source

            if self.auto_mode_switching:
                self.update_mode(source)

    def calculate_tag_distance(self, pose):
        return math.sqrt(
            pose.position.x ** 2 +
            pose.position.y ** 2 +
            pose.position.z ** 2
        )

    def update_mode(self, source):
        self.get_logger().info(f'Mode updated based on tag from: {source}')

    def publish_april_destination(self, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tag_frame
        msg.pose = pose
        self.destination_pub.publish(msg)

    def publish_person_detected(self, status: bool):
        self.person_back_pub.publish(Bool(data=status))

    def publish_no_tag_detected(self):
        self.no_tag_pub.publish(Bool(data=True))
        self.get_logger().warn('No AprilTag detected recently or tag out of range.')

    def tag_detection_timer(self):
        now = self.get_clock().now()
        if self.last_detection_time is None:
        # No detection ever received or after reset
            self.publish_no_tag_detected()
            return

        time_diff_sec = (now - self.last_detection_time).nanoseconds / 1e9
        if time_diff_sec > self.no_tag_timeout:
            self.publish_no_tag_detected()
            self.last_detection_time = None


def main(args=None):
    import rclpy
    from april_tag_monitor.tag_monitor_node import TagMonitorNode  # adjust import if needed

    rclpy.init(args=args)
    node = TagMonitorNode()
    source = node.last_detection_source
    if source == 'front':
        self.publish_mode('lead_ahead')
    elif source == 'rear':
        self.publish_mode('follow_behind')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
