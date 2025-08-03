import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from grid_planners_demo.planners.astar import AStar
from grid_planners_demo.collision_checker import CollisionChecker
from grid_planners_demo.node import Node
import tf2_ros
import time

class NavigatorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('online', True)
        self.declare_parameter('use_path_smoother', True)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('goal_tolerance', 0.1)

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.online = self.get_parameter('online').get_parameter_value().bool_value
        self.use_path_smoother = self.get_parameter('use_path_smoother').get_parameter_value().bool_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.map_ = None
        self.map_resolution_ = None
        self.map_origin_ = None
        self.map_size_ = None
        self.robot_position_ = None
        self.robot_orientation_ = None
        self.goal_position_ = None
        self.goal_orientation_ = None

        self.map_subscription_ = self.create_subscription(OccupancyGrid,
                                                        '/map',
                                                        self.map_callback,
                                                        10)
        self.map_subscription_

        self.goal_query_subscription_ = self.create_subscription(PoseStamped,
                                                                goal_topic,
                                                                self.goal_query_callback,
                                                                10)
        self.goal_query_subscription_

        self.path_publisher_ = self.create_publisher(Path,
                                                '/plan',
                                                10)
        self.status_publisher_ = self.create_publisher(Int32,
                                                    '/path_follower/status',
                                                    10)

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)
        self.get_robot_pose()
        self.get_logger().info('Initialised')

    def map_callback(self, msg):
        self.map_ = msg
        self.map_resolution_ = self.map_.info.resolution
        self.map_origin_ = self.map_.info.origin
        self.map_size_ = (self.map_.info.width, self.map_.info.height)
        self.get_logger().info('Map received')

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.robot_position_ = (transform.transform.translation.x, transform.transform.translation.y)
            self.robot_orientation_ = transform.transform.rotation
            return self.robot_position_, self.robot_orientation_
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            return None, None

    def goal_query_callback(self, msg):
        self.goal_position_ = (msg.pose.position.x, msg.pose.position.y)
        self.goal_orientation_ = msg.pose.orientation
        self.get_logger().info('Received new goal')
        return self.planning_callback()

    def planning_callback(self):
        if self.map_ is None:
            self.get_logger().info('No map received yet')
            return

        self.get_logger().info('Starting planner')
        msg = Int32()
        msg.data = 1
        self.status_publisher_.publish(msg)

        # --- THIS IS THE CRITICAL FIX ---
        # Wait until we have a valid robot position before planning
        while self.robot_position_ is None:
            self.get_logger().info("Waiting for initial robot pose...")
            self.get_robot_pose()
            time.sleep(0.5)
        # --- END OF FIX ---

        start = Node.from_tf(self.robot_position_, self.robot_orientation_)
        goal = Node.from_tf(self.goal_position_, self.goal_orientation_)
        start.parent = None
        goal.parent = None

        if self.online:
            self.collision_checker_ = CollisionChecker(self.map_, self.robot_radius)

        self.planner_ = AStar(start, goal, self.collision_checker_, self.map_size_, self.map_resolution_, self.map_origin_)
        self.planner_.set_goal_tolerance(self.goal_tolerance)
        start_time = time.time()
        path = self.planner_.plan()
        end_time = time.time()
        if path is None:
            self.get_logger().info('No path found')
            msg = Int32()
            msg.data = -1
            self.status_publisher_.publish(msg)
            return

        self.get_logger().info(f'Path found in {end_time - start_time} seconds')

        if self.use_path_smoother:
            self.get_logger().info('Smoothing path')
            path = self.planner_.smooth_path(path)
            self.get_logger().info('Path smoothed')

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.orientation = self.goal_orientation_
            path_msg.poses.append(pose)
        self.path_publisher_.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorNode()
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()