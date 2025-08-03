import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import yaml

class MissionHandler(Node):
    def __init__(self):
        super().__init__('mission_handler_node')

        self.declare_parameter('waypoints_file', '')
        waypoints_file_path = self.get_parameter('waypoints_file').get_parameter_value().string_value

        if not waypoints_file_path:
            self.get_logger().error('Waypoints file path not provided!')
            return

        self.get_logger().info(f'Loading waypoints from: {waypoints_file_path}')

        try:
            with open(waypoints_file_path, 'r') as file:
                self.waypoints = yaml.safe_load(file)['waypoints']
            self.get_logger().info(f'Successfully loaded {len(self.waypoints)} waypoints.')
        except (yaml.YAMLError, FileNotFoundError) as e:
            self.get_logger().error(f"Error loading waypoints file: {e}")
            self.waypoints = []
            return

        # State variable to track if we are ready for a new goal
        self.ready_to_send_goal = True

        # Publisher for sending goals
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("Publisher on /goal_pose topic created.")

        # Subscriber for the path follower's status
        self.status_subscriber = self.create_subscription(
            Int32,
            '/path_follower/status',
            self.status_callback,
            10)
        self.get_logger().info("Subscriber on /path_follower/status topic created.")

        # Use a timer to check and send the first goal
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.ready_to_send_goal:
            self.send_next_goal()
            self.timer.cancel() # Stop the timer after sending the first goal

    def status_callback(self, msg):
        # Status code 2 means "succeeded"
        if msg.data == 2 and not self.ready_to_send_goal:
            self.get_logger().info('Goal reached successfully!')
            self.ready_to_send_goal = True
            self.send_next_goal()
        elif msg.data == 0: # Status code 0 means "idle"
             self.ready_to_send_goal = True

    def send_next_goal(self):
        if not self.waypoints:
            self.get_logger().info('All waypoints have been sent. Mission complete!')
            return

        if self.ready_to_send_goal:
            self.ready_to_send_goal = False # Mark as busy
            waypoint = self.waypoints.pop(0)
            self.get_logger().info(f"Sending goal: {waypoint['name']}")

            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            pose_data = waypoint['pose']
            goal_msg.pose.position.x = float(pose_data['position']['x'])
            goal_msg.pose.position.y = float(pose_data['position']['y'])
            goal_msg.pose.orientation.z = float(pose_data['orientation']['z'])
            goal_msg.pose.orientation.w = float(pose_data['orientation']['w'])

            self.goal_publisher.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()