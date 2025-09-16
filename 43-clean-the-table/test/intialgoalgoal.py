import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan

class StartInspection(Node):
    def __init__(self):
        super().__init__('start_inspection')  # Node name
        self.get_logger().info('start_inspection Node has started')

        # Create initial position publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Goal publisher
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        

        
        # Wait for the action server
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('waiting for action server')       
        
        # Create a subscriber for laser scan data
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Replace with the actual topic name
            self.laser_scan_callback,
            10
        )

        # Initialize variables
        self.door_is_open = False
        self.initial_pose_published = False
        self.movement_complete = False
        self.min_distance_threshold = 0.5  # Minimum distance to consider the door open
        self.goal_published = False  # Flag to track if the goal has been published
        self.door_open_detected = False  # Flag to track if the door is detected as open

        # Publish initial pose once on node startup
        self.publish_initial_pose()
    

    def laser_scan_callback(self, msg):
        # Check for open space in front of the robot (example)
        min_distance = min(msg.ranges)
        if min_distance > 0.5:  # Adjust this threshold as needed
            self.door_is_open = True 
               
            self.get_logger().info("Door detected open, preparing to move robot.")
            self.publish_goal() # Move robot linearly for 10 seconds
        


    def publish_initial_pose(self):
        if not self.initial_pose_published:
            initial = PoseWithCovarianceStamped()

            initial.header.frame_id = 'map'
            initial.header.stamp = self.get_clock().now().to_msg()

            # Initial position
            initial.pose.pose.position.x = -7.22
            initial.pose.pose.position.y = 1.2578334448505635
            initial.pose.pose.orientation.z = 0.0469387441221786
            initial.pose.pose.orientation.w = 0.9988977696942929

            # Publish initial pose
            self.pose_publisher.publish(initial)
            self.get_logger().info(f"Published initial pose: x={initial.pose.pose.position.x}, y={initial.pose.pose.position.y}")
            self.initial_pose_published = True  # Mark initial pose as published

    def publish_goal(self):
        if not self.goal_published:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()

            # Goal position coordinates
            goal.pose.position.x = -0.405  # Target position (x)
            goal.pose.position.y =  5.4614  # 5.4614 Target position (y)
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = -0.12
            goal.pose.orientation.w = 0.9986
     #x:=-4.05 y:=2.26 yaw:=1.61
            # Publish the goal
            self.goal_publisher_.publish(goal)
            self.get_logger().info(f'Published goal: x={goal.pose.position.x}, y={goal.pose.position.y}')

            # Create a goal for the navigation action
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal



            # Send the goal to the navigation action server
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)

            # Wait for the result
            self._send_goal_future.add_done_callback(self._goal_response_callback)
            

            self.goal_published = True  # Set the flag to True after publishing the goal

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected!')
            return

        self.get_logger().info('Goal was accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            result = future.result().result
            if hasattr(result, 'navigation_status'): 
                if result.navigation_status == NavigateToPose.Result.SUCCEEDED:
                    self.get_logger().info('Goal succeeded!')
                else:
                    self.get_logger().info(f'Goal failed with status: {result.navigation_status}')
            else:
                self.get_logger().warn("No 'navigation_status' attribute found in result.")
        except AttributeError:
            self.get_logger().error("Error accessing result attributes.")

def main(args=None):
    rclpy.init(args=args)
    print("Node is running")  # Debug line
    start_inspection = StartInspection()
    rclpy.spin(start_inspection)
    start_inspection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



