import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot
from pick_and_place_interfaces.srv import Pickandplace # Ensure you have this service defined
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

# for initial pose and goal pose import module 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan

import numpy as np
import math



class Pickandplaceobject(Node):
    def __init__(self):
        super().__init__("pick_and_place_object")

        # for moving the robot       
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Going to initial position algorithm for intial pose to goal pose 1

        self.get_logger().info('First step moving to initial position')

        # Create initial position publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Goal publisher
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Velocity publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
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

        # Marker position and further task 

        self.get_logger().info('Now starting too detect 3 marker ')


        # TF Listener to track marker pose
        self.source_frame_1 = "tag36h11:1"  # Marker frame
        self.marker_id_1 = 1

        #for second object 
        self.source_frame_2 = "tag36h11:2"  # Marker frame
        self.marker_id_2 = 2
        #for third object
        self.source_frame_3 = "tag36h11:3"  # Marker frame
        self.marker_id_3 = 3

        #for forth object surface object
        #self.source_frame_10 = "tag36h11:10"  # Marker frame
        #self.marker_id_10 = 10

        self.target_frame = "base_footprint"  # Robot base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pose = None  # To store the latest marker pose
        self.goal_reached = False
        #adding additional parameter
        self.__synchronous = True
        self.__cartesian = False
        self.__cartesian_max_step = 0.0025
        self.__cartesian_fraction_threshold = 0.0
        # -----------Create service to handle requests
        self.timer_count = 0
        # which call through client member function and srv msg cmake folder (run through # ros2 run pick_and_place_service client)
        self.service = self.create_service(Pickandplace, 'pick_and_place_object', self.handle_pick_and_place_request)
        self.box_added = False  # Flag to track if box is added
        self.obj_1_lift = False # object one succesful
        self.obj_2_lift = False # object one succesful
        self.obj_3_lift = False # object one succesful


        self.move = False

        # Initialize pose attributes with default values
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.rot_w = 1.0

        # Keep track of the last pose to detect changes
        self.last_pose = None

        # MoveIt2 interface
        self.__callback_group = ReentrantCallbackGroup()  # Callback group
        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )
        self.get_logger().info("MoveIt2 initialized. Waiting for readiness...")


# adding additional parameter

        self.__moveit2.planner_id = "RRTConnectkConfigDefault" 
        self.__moveit2.max_velocity = 0.2 # 0.5
        self.__moveit2.max_acceleration = 0.2 #0.5
        self.__moveit2.cartesian_avoid_collisions = False
        self.__moveit2.cartesian_jump_threshold = 0.0
        self.wristPose = None

# gripper interface function

        self.__gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self.__callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )
        #self.tagPose = TagPose

        # Timer to update marker pose and manage collision objects
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer for periodic updates

        self.get_logger().info("AddCollisionObjectService initialized.")



##### changing laserscan from here to timere 

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
            '''goal.pose.position.x = -0.405  # Target position (x)
            goal.pose.position.y =  5.4614  # 5.4614 Target position (y)
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = -0.12
            goal.pose.orientation.w = 0.9986 #   ig coordinate'''
     #x:=-4.05 y:=2.26 yaw:=1.61


            # 
            goal.pose.position.x = -4.05  # Updated Target position (x)
            goal.pose.position.y =  2.26  # Updated Target position (y)
            goal.pose.position.z = 0.0    # Assuming it's on the ground

            goal.pose.orientation.z = 0.72  # Updated yaw (converted to quaternion)
            goal.pose.orientation.w = 0.69  # Updated yaw (converted to quaternion)





            # Publish the goal
            self.goal_publisher_.publish(goal)
            self.get_logger().info(f'Published goal: x={goal.pose.position.x}, y={goal.pose.position.y}')

            # Create a goal for the navigation action
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal

            # Set the maximum velocity to 0.5 m/s
            goal_msg.max_velocity = 0.5  # Set the speed to 0.5 m/s

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



    #   timer call back for marker and motion

    def timer_callback(self):
        try:
            for marker_id, source_frame in {
                self.marker_id_1: self.source_frame_1,
                self.marker_id_2: self.source_frame_2,
                self.marker_id_3: self.source_frame_3,
                #self.marker_id_10: self.source_frame_10,
            }.items():
                if not self.tf_buffer.can_transform(self.target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                    self.get_logger().info(f"Waiting for transform {source_frame} -> {self.target_frame}")
                    return

            # Get transforms for all objects
            transform_1 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_1, rclpy.time.Time())
            transform_2 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_2, rclpy.time.Time())
            transform_3 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_3, rclpy.time.Time())
            #transform_10 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_10, rclpy.time.Time())

            # Extract position and orientation
            def extract_pose(transform):
                return {
                    "pos": [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z],
                    "rot": [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w],
                }

            marker_pose_1 = extract_pose(transform_1)
            marker_pose_2 = extract_pose(transform_2)
            marker_pose_3 = extract_pose(transform_3)
            #marker_pose_10 = extract_pose(transform_10)

            self.get_logger().info(f"Object 1 Position: {marker_pose_1['pos']} Orientation: {marker_pose_1['rot']}")
            self.get_logger().info(f"Object 2 Position: {marker_pose_2['pos']} Orientation: {marker_pose_2['rot']}")
            self.get_logger().info(f"Object 3 Position: {marker_pose_3['pos']} Orientation: {marker_pose_3['rot']}")
            #self.get_logger().info(f"Surface Object Position: {marker_pose_10['pos']} Orientation: {marker_pose_10['rot']}")

            # Open the gripper
            self.__open_gripper()

            # Convert orientation (quaternion to Euler and fix)
            tagEuler = self.euler_from_quaternion(marker_pose_1["rot"])
            fixed_tagEuler = (tagEuler[0] - 90, tagEuler[1], tagEuler[2] - 90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            # Add collision box   y right means -  and left +
            if not self.box_added:
                self.__add_collision_box(
                    object_id=f"collision_table_{self.marker_id_1}",
                    position=[marker_pose_1["pos"][0] + 0.21, marker_pose_1["pos"][1] - 0.11, marker_pose_1["pos"][2] - 0.067],
                    orientation=marker_pose_1["rot"],
                    dimensions=[0.5, 0.5, 0.1]
                )

            # adding collision box 1

                self.__add_collision_object_1(
                    object_id=f"collision_obj_{self.marker_id_1}",
                    position=[marker_pose_1["pos"][0] + 0.065, marker_pose_1["pos"][1], marker_pose_1["pos"][2]+0.078],
                    orientation=marker_pose_1["rot"],
                    dimensions=[0.04, 0.06, 0.15]
                )

            # adding collision box 2

                self.__add_collision_object_2(
                    object_id=f"collision_obj_{self.marker_id_2}",
                    position=[marker_pose_2["pos"][0] + 0.07, marker_pose_1["pos"][1]- 0.06, marker_pose_1["pos"][2]+0.078],
                    orientation=marker_pose_2["rot"],
                    dimensions=[0.04, 0.06, 0.15]
                )
            # adding collision box 3

                self.__add_collision_object_3(
                    object_id=f"collision_obj_3_used_wit_offset{self.marker_id_2}",
                    position=[marker_pose_2["pos"][0] - 0.01, marker_pose_2["pos"][1] - 0.16, marker_pose_2["pos"][2]+0.078],
                    orientation=marker_pose_2["rot"],
                    dimensions=[0.04, 0.06, 0.15]
                )

                self.box_added = True



### code for moving the robot hand 


# now first object 
            # Define movement positions for object 1
            position_1 = [marker_pose_1["pos"][0] - 0.11, marker_pose_1["pos"][1], marker_pose_1["pos"][2] + 0.08]
            lift_position_1 = [position_1[0], position_1[1], position_1[2] + 0.3]
            new_position_1 = [position_1[0], position_1[1] - 0.1, position_1[2]]


            # Define movement positions for object 2
            position_2 = [marker_pose_2["pos"][0] - 0.11, marker_pose_2["pos"][1], marker_pose_2["pos"][2] + 0.08]
            lift_position_2 = [position_2[0], position_2[1], position_2[2] + 0.3]
            new_position_2 = [position_2[0], position_2[1] - 0.1, position_2[2]]


            # Define movement positions for object 3
            position_3 = [marker_pose_3["pos"][0] - 0.11, marker_pose_3["pos"][1], marker_pose_3["pos"][2] + 0.08]
            lift_position_3 = [position_3[0], position_3[1], position_3[2] + 0.3]
            new_position_3 = [position_3[0], position_3[1] - 0.1, position_3[2]]

###### movement object 1 coding 

            # Move the arm
            self.get_logger().info("Moving arm to pose")
            self.__move_arm_to_pose(position=position_1, orientation=fixed_tagQuat)

            if self.timer_count >= 3:
                self.__remove_collision_object_1(object_id=f"collision_obj_{self.marker_id_1}")
                self.get_logger().info('Box Collider removed')

                # Partially close gripper
                self.get_logger().info('Partially closing Gripper')
                self.__partial_close_gripper()
                time.sleep(3)

                # Lift object
                self.get_logger().info('Lifting object')
                self.__moveit2.max_velocity = 0.2
                self.__moveit2.max_acceleration = 0.2
                self.__move_arm_to_pose(position=lift_position_1, orientation=fixed_tagQuat)
                time.sleep(3)

                # moving to new position
                self.get_logger().info('MOvingto new position')
                self.__move_arm_to_pose(position=new_position_1, orientation=fixed_tagQuat)


                # Open gripper to release the object
                self.get_logger().info('Releasing object')
                self.__open_gripper()
                time.sleep(3)

                # Move arm back to a safe position
                self.get_logger().info('Moving arm to safe position')
                safe_position = [lift_position_1[0], lift_position_1[1], lift_position_1[2] + 0.2]
               # self.__move_arm_to_pose(position=safe_position, orientation=fixed_tagQuat)
                #time.sleep(3)

            # linearly move 
############## line removed here add                            


                # Destroy the timer after execution
                self.get_logger().info('Destroying the timer after 3 executions.')
                self.destroy_timer(self.timer)



        except TransformException as ex:
            self.get_logger().info(f"Could not transform {self.source_frame_1} to {self.target_frame}: {ex}")


# function to move robot

    def move_robot_right(self):
        """Moves the robot to the right (positive Y direction) with velocity 0.5 for 7 seconds."""
        
        # Create a Twist message
        msg = Twist()
        msg.linear.y = 0.5  # Move in the positive Y direction (right)
        msg.angular.z = 0.0  # No rotation

        # Publish the velocity command
        self.get_logger().info("Moving robot to the right for 7 seconds.")
        self.publisher_.publish(msg)

        # Keep moving for 7 seconds
        time.sleep(7)

        # Stop the robot by setting velocity to zero
        msg.linear.y = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Robot stopped.")




    # function for adding the collision box below the table 
    def __add_collision_box(self, object_id: str, position: list, orientation: list, dimensions: list):
        """
        Add a Box shape as a collision box

        Args:
        - object_id: unique object name
        - position: 3D position (x, y, z) relative to base footprint
        - orientation: quaternion orientation (x, y, z, w) relative to base footprint
        - dimensions: dimension of the box
        """
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")

    # definition for object 1
        
    def __add_collision_object_1(self, object_id: str, position: list, orientation: list, dimensions: list):
   
        """
        Add a Box shape as a collision box

        Args:
        - object_id: unique object name
        - position: 3D position (x, y, z) relative to base footprint
        - orientation: quaternion orientation (x, y, z, w) relative to base footprint
        - dimensions: dimension of the box
        """
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")   




    # definition for object 2
        
    def __add_collision_object_2(self, object_id: str, position: list, orientation: list, dimensions: list):
   
        """
        Add a Box shape as a collision box

        Args:
        - object_id: unique object name
        - position: 3D position (x, y, z) relative to base footprint
        - orientation: quaternion orientation (x, y, z, w) relative to base footprint
        - dimensions: dimension of the box
        """
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")   

    
    # definition for object 3
        
    def __add_collision_object_3(self, object_id: str, position: list, orientation: list, dimensions: list):
   
        """
        Add a Box shape as a collision box

        Args:
        - object_id: unique object name
        - position: 3D position (x, y, z) relative to base footprint
        - orientation: quaternion orientation (x, y, z, w) relative to base footprint
        - dimensions: dimension of the box
        """
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")   




    # function for moving the arm to required position
    
    def __move_arm_to_pose(self, position:list[float, float, float], orientation:list[float, float, float, float]):
        """
        Move the arm to a given pose relative of base footprint

        Args:
        - position: 3D position (x,y,z) relative of base footprint
        - orientation: quaternion orientation (x, y, z, w) relative of base footprint
        """
        
        self.__moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            cartesian=self.__cartesian,
            cartesian_max_step=self.__cartesian_max_step,
            cartesian_fraction_threshold=self.__cartesian_fraction_threshold,
        )
        self.move = True
        if self.__synchronous: 
            self.__moveit2.wait_until_executed()

    
    
     # open and close gripper call inside timer
    def __open_gripper(self):
        """
        Open gripper of tiago
        """
        self.__gripper_interface.open()
        self.__gripper_interface.wait_until_executed()




    def __partial_close_gripper(self):
        """
        Close gripper of tiago
        """
        partial_close_position = 0.03 #0.0195  the width of box is 0.04
        #partial_close_positions = 0.3  # Example values for partial closure
        self.__gripper_interface.move_to_position(partial_close_position)
        self.__gripper_interface.wait_until_executed()




    def __remove_collision_object_1(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)

 # removing object 2

    def __remove_collision_object(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)

# removing object3


    def __remove_collision_object(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)



        # while moving remove all objecte 


    def handle_pick_and_place_request(self, request, response):
        """Handles requests to add a collision object.
        The client command will log the message of the service
        it takes marker_id as input
        and return response
        client can be called by (ros2_ws collision_service client)"""
        self.get_logger().info(f"Received request to add collision object for marker ID: {request.marker_id}")

        # Wait for the command by client to trigger collision box addition
        if self.move == True:
            self.get_logger().info("Successfully movement planned object grasp and moved.")
            response.success = True
        else:
            self.get_logger().info("Collision box not added yet.")
            response.success = False

        return response





    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert results to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        return roll_deg, pitch_deg, yaw_deg

   

    def quaternion_from_euler(self,euler_angles):
        """
        Converts Euler angles (roll, pitch, yaw) in degrees to a quaternion (x, y, z, w).
    
        Args:
            roll: Rotation around the X-axis in degrees.
            pitch: Rotation around the Y-axis in degrees.
            yaw: Rotation around the Z-axis in degrees.
    
        Returns:
            A list [x, y, z, w] representing the quaternion.
        """

        # Extract roll, pitch, and yaw from the input list
        roll, pitch, yaw = euler_angles
        # Convert degrees to radians
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        # Perform the quaternion calculations
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # Ensure the quaternion is in [x, y, z, w] order
        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy  # w
        q[1] = sr * cp * cy - cr * sp * sy  # x
        q[2] = cr * sp * cy + sr * cp * sy  # y
        q[3] = cr * cp * sy - sr * sp * cy  # z

        #    Swap to [x, y, z, w]
        return [q[1], q[2], q[3], q[0]]


	

def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    # Initialize the AddCollisionObjectService node
    pick_and_place_object_node= Pickandplaceobject()
    executor.add_node(pick_and_place_object_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        pick_and_place_object_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#ros2_ws/src/nb-246830/stage_6/41-pick-and-place-object/pick_and_place/pick_and_place