import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class RealSense(Node):
    def __init__(self):
        super().__init__('realsense')
        
        # Initialize storage variables
        self.position = None
        self.orientation = None
        self.timestamp = None
        self.frame_id = None

        # Initialize SENDING storage variables
        self.set_position = None
        self.set_orientation = None

        # Subscriber to RealSense pose data
        self.realsense_subscriber = self.create_subscription(PoseStamped, '/camera/pose/sample', self.realsense_callback, 1)
        self.get_logger().info('Subscribing to RealSense!')

        # Publisher for VisionPose topic
        self.vision_pose_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 1)
        self.get_logger().info('Publishing to VisionPose')

        # Statement to end the inits
        self.get_logger().info('Realsense Node All Setup and Started!')


    def realsense_callback(self, msg):
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.timestamp = msg.header.stamp
        self.frame_id = msg.header.frame_id
        
        # Everytime we get stuff, write both immediately
        self.send_vision_pose()


    def send_vision_pose(self):
        # Create a new PoseStamped message to publish to vision_pose topic
        vision_pose_msg = PoseStamped()
        vision_pose_msg.header.stamp = self.timestamp
        vision_pose_msg.header.frame_id = self.frame_id
        vision_pose_msg.pose.position = self.position
        vision_pose_msg.pose.orientation = self.orientation
        # Publish the message to the /mavros/vision_pose/pose topic
        self.vision_pose_publisher.publish(vision_pose_msg)

        # Print values normally
        print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        print(f"Frame ID: {self.frame_id}")



def main(args=None):
    rclpy.init(args=args)
    node = RealSense()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
