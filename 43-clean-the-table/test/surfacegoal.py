import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan

class StartInspection(Node): # start inspection class created
    def __init__(self):
        super().__init__('start_inspection') # giving the node name
        self.get_logger().info('start_inspection Node has started')

        #create initial position publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped,'/initialpose', 10)
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_publisher_=self.create_publisher(PoseStamped,'/goal_pose',10)
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('waiting for action server')       
        
        
        # Create a subscriber for laser scan data
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with the actual topic name
            self.laser_scan_callback,
            10
        )

        self.door_is_open = False
        self.timer = self.create_timer(2.0, self.timer_callback) 

    def timer_callback(self):
        if self.door_is_open:
            self.publish_goal() 

    def laser_scan_callback(self, msg):
        # Check for open space in front of the robot (example)
        min_distance = min(msg.ranges)
        if min_distance > 0.5:  # Adjust this threshold as needed
            self.door_is_open = True 

    def timer_callback(self):
        self.publish_goal()
        self.publish_initial_pose()

# add initialpose


    def publish_initial_pose(self):
        initial = PoseWithCovarianceStamped()


        #initial position
        #initial.pose.pose.position.x = -7.242931365966797
        #initial.pose.pose.position.y = 1.3187459707260132
        # giving orientation
        #initial.pose.pose.orientation.w = 1.0
        #self.get_logger().info(f"Published initial pose: x={initial.pose.pose.position.x}, y={initial.pose.pose.position.y}")

        #initial.header.frame_id = 'map'
        #initial.header.stamp = self.get_clock().now().to_msg()

        # Initial position
        #initial.pose.pose.position.x = -7.242931365966797
        #initial.pose.pose.position.y = 1.3187459707260132 
        #initial.pose.pose.position.z = 0.0 

        # Initial orientation (facing forward)

        #initial.pose.pose.orientation.w = 1.0

    # goal position functions

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        #position goal coordinate

        goal.pose.position.x = -0.4245590894594296
        goal.pose.position.y = -0.7484545436286829
        goal.pose.position.z = 0.0
        #goal.pose.position.y = -0.5025840997695923

        #giving orientation

        goal.pose.orientation.z = 0.0028730754028709854
        goal.pose.orientation.w = 0.9999958727103474


        # publish the goal
        self.goal_publisher_.publish(goal)
        self.get_logger().info(f'Publishing goal: x={goal.pose.position.x}, y={goal.pose.position.y}')
        
        # Create a goal for the navigation action
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        # Send the goal to the navigation action server
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Wait for the result
        self._send_goal_future.add_done_callback(self._goal_response_callback)

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
# setting script beginning point client library prerequisite setting
def main(args=None):
    rclpy.init(args=args)
    print("Node is running")  # Debug line
    start_inspection = StartInspection()
    rclpy.spin(start_inspection)
    start_inspection .destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

