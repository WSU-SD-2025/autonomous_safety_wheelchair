import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import math

class LeadAheadNode(Node):
    def __init__(self):
        super().__init__('lead_ahead_node')

        # Declare and load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_distance_m', 1.0),
                ('max_speed_mps', 0.8),
                ('deceleration_mps2', 0.35),
                ('linear_kp', 0.6),
                ('angular_kp', 0.6),
                ('control_period_s', 0.1),
                ('april_destination_topic', '/april_destination'),
                ('no_tag_topic', '/no_tag_detected'),
                ('cmd_vel_topic', '/cmd_vel'),
            ]
        )

        self.desired_distance = self.get_parameter('desired_distance_m').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        self.deceleration = self.get_parameter('deceleration_mps2').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.control_period = self.get_parameter('control_period_s').value

        self.april_destination_topic = self.get_parameter('april_destination_topic').value
        self.no_tag_topic = self.get_parameter('no_tag_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Internal state
        self.current_speed = 0.0
        self.no_tag = True
        self.last_seen_time = self.get_clock().now()

        
        # subscribe to topics and create publisher
        
        self.create_subscription(PoseStamped, self.april_destination_topic, self.april_cb, 10)
        self.create_subscription(Bool, self.no_tag_topic, self.no_tag_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Control loop timer
        self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info('LeadAheadNode initialized and running.')

    # Callback: AprilTag detected — adjust velocity to maintain distance
    def april_cb(self, msg: PoseStamped):
        self.no_tag = False
        self.last_seen_time = self.get_clock().now()

        # Compute distance between wheelchair and tag
        distance = math.sqrt(
            msg.pose.position.x ** 2 +
            msg.pose.position.y ** 2 +
            msg.pose.position.z ** 2
        )

        # Proportional control for forward motion
        error = distance - self.desired_distance
        target_speed = self.linear_kp * error

        # adjust speed limits
        target_speed = max(min(target_speed, self.max_speed), -self.max_speed)
        self.current_speed = target_speed

        # change angle if needed
        lateral_offset = msg.pose.position.y
        angular_z = -self.angular_kp * lateral_offset

        # Publish 
        self.publish_cmd(self.current_speed, angular_z)

    # Callback: No tag detected flag from AprilTag monitor
    def no_tag_cb(self, msg: Bool):
        if msg.data:
            if not self.no_tag:
                self.get_logger().warn('AprilTag lost slowing to stop.')
            self.no_tag = True

    # Timer: Control loop for gradual deceleration when tag lost
    def control_loop(self):
        if self.no_tag:
            # Gradually decelerate to stop
            dt = self.control_period
            if abs(self.current_speed) > 0.0:
                decel = self.deceleration * dt
                if self.current_speed > 0:
                    self.current_speed = max(0.0, self.current_speed - decel)
                else:
                    self.current_speed = min(0.0, self.current_speed + decel)

                self.publish_cmd(self.current_speed, 0.0)

    # Helper: Publish Twist command change to be more suited with twist mux 
    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LeadAheadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
