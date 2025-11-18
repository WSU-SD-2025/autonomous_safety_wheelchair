import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf_transformations import euler_from_quaternion

# The name of the actor model in Gazebo, from spawn_gazebo.launch.py
ACTOR_MODEL_NAME = 'actor_robot' 

class CaregiverPosPublisher(Node):

    def __init__(self):
        super().__init__('caregiver_pos_publisher')
        
        # Publisher for the final output topic
        self.publisher_ = self.create_publisher(
            PoseStamped, 
            '/caregiver_pos', 
            10
        )
        
        #Subscriber to the bridged Gazebo ModelStates topic
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model/pose', 
            self.model_states_callback,
            10
        )
        self.get_logger().info('Caregiver Position Publisher Node Initialized.')

    def model_states_callback(self, msg: ModelStates):
        """
        Callback to process the ModelStates message.
        """
        try:
            # Find the index
            idx = msg.name.index(ACTOR_MODEL_NAME)
        except ValueError:
            # Model not found 
            return

        # Get the pose of the actor model
        actor_pose = msg.pose[idx]

        # Create PoseStamped message
        pose_stamped_msg = PoseStamped()
        
        # Header 
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'odom' 
        
        # Pose: Position (x, y, z)
        pose_stamped_msg.pose.position.x = actor_pose.position.x
        pose_stamped_msg.pose.position.y = actor_pose.position.y
        pose_stamped_msg.pose.position.z = actor_pose.position.z
        
        # Pose: Orientation 
        pose_stamped_msg.pose.orientation = actor_pose.orientation
        
        # Publish the final PoseStamped message
        self.publisher_.publish(pose_stamped_msg)
        self.get_logger().debug(f'Published position: x={actor_pose.position.x:.2f}, y={actor_pose.position.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    caregiver_pos_publisher = CaregiverPosPublisher()
    rclpy.spin(caregiver_pos_publisher)
    caregiver_pos_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()