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
from pick_and_place_interfaces.srv import Pickandplace
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class Pickandplaceobject(Node):
    def __init__(self):
        super().__init__("pick_and_place_object")

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # for linear movement
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)  # for initial position
        self.goal_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.laser_scan_callback, 10)

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF Listener
        self.source_frame_1 = "tag36h11:1"
        self.source_frame_2 = "tag36h11:2"
        self.source_frame_3 = "tag36h11:3"
        self.target_frame = "base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize variables
        self.door_is_open = False
        self.initial_pose_published = False
        self.movement_complete = False
        self.min_distance_threshold = 0.5
        self.goal_published = False
        self.door_open_detected = False  # Corrected typo in variable name
        self.box_added = False
        self.obj_1_lift = False
        self.obj_2_lift = False
        self.obj_3_lift = False
        self.move = False
        self.timer_count = 0

        # MoveIt2 and Gripper Interface
        self.__callback_group = ReentrantCallbackGroup()
        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )
        self.__gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self.__callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Additional attributes
        self.__cartesian = False
        self.__cartesian_max_step = 0.0025
        self.__cartesian_fraction_threshold = 0.0
        self.__synchronous = True
        self.marker_id_1 = 1

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)

    def extract_pose(self, transform):
        """
        Extract position and orientation from a transform.
        """
        return {
            "pos": [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z],
            "rot": [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w],
        }

    def laser_scan_callback(self, msg):
        """
        Callback function for processing laser scan data.
        Detects if the door is open based on the minimum distance threshold.
        """
        min_distance = min(msg.ranges)  # Get the minimum distance from the laser scan data
        if min_distance > self.min_distance_threshold:
            self.door_open_detected = True
            self.get_logger().info(f"Door is open! Minimum distance: {min_distance:.2f} meters")
        else:
            self.door_open_detected = False
            self.get_logger().info(f"Door is closed. Minimum distance: {min_distance:.2f} meters")

    def publish_initial_pose(self):
        if not self.initial_pose_published:
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id = 'map'
            initial.header.stamp = self.get_clock().now().to_msg()
            initial.pose.pose.position.x = -7.22
            initial.pose.pose.position.y = 1.2578334448505635
            initial.pose.pose.orientation.z = 0.0469387441221786
            initial.pose.pose.orientation.w = 0.9988977696942929
            self.pose_publisher.publish(initial)
            self.get_logger().info(f"Published initial pose: x={initial.pose.pose.position.x}, y={initial.pose.pose.position.y}")
            self.initial_pose_published = True


    def create_goal_pose(self, x, y, z, rot_z, rot_w):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.z = rot_z
        goal.pose.orientation.w = rot_w
        return goal.pose


    def publish_goal(self, goal.pose):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = goal.pose
        self.goal_publisher_.publish(goal)
        self.get_logger().info(f'Published goal: x={goal.pose.position.x}, y={goal.pose.position.y}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        goal_msg.max_velocity = 0.5
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        self.goal_published = True

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

    def timer_callback(self):
        """
        Timer callback to continuously monitor the door status and handle marker detection.
        """
        self.timer_count += 1  # Increment the timer count
        self.publish_initial_pose()
        self.get_logger().info("Waiting for door to open...")

        # Check if the door is open
        if self.door_open_detected == True:
                goal_pose = self.create_goal_pose(-4.05, 2.26, 0.0, 0.72, 0.69)  # x, y, z, rot_z, rot_w
                self.publish_goal(goal_pose)

        # Handle marker detection
        self.handle_marker_detection()

    def handle_marker_detection(self):
        """
        Handle marker detection and object manipulation.
        """
        try:
            # Check if transforms are available
            for marker_id, source_frame in {
                1: self.source_frame_1,
                2: self.source_frame_2,
                3: self.source_frame_3,
            }.items():
                if not self.tf_buffer.can_transform(self.target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                    self.get_logger().warn(f"Waiting for transform {source_frame} -> {self.target_frame}")
                    return  # Skip this iteration if transform is not available

            # Get transforms for all markers
            transform_1 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_1, rclpy.time.Time())
            transform_2 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_2, rclpy.time.Time())
            transform_3 = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame_3, rclpy.time.Time())

            # Extract poses
            marker_pose_1 = self.extract_pose(transform_1)
            marker_pose_2 = self.extract_pose(transform_2)
            marker_pose_3 = self.extract_pose(transform_3)

            self.get_logger().info(f"Object 1 Position: {marker_pose_1['pos']} Orientation: {marker_pose_1['rot']}")
            self.get_logger().info(f"Object 2 Position: {marker_pose_2['pos']} Orientation: {marker_pose_2['rot']}")
            self.get_logger().info(f"Object 3 Position: {marker_pose_3['pos']} Orientation: {marker_pose_3['rot']}")

            # Open the gripper
            self.__open_gripper()

            # Convert orientation (quaternion to Euler and fix)
            tagEuler = self.euler_from_quaternion(marker_pose_1["rot"])
            fixed_tagEuler = (tagEuler[0] - 90, tagEuler[1], tagEuler[2] - 90)
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)

            # Add collision boxes if not already added
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
                

            # Define movement positions for object 1
            position_1 = [marker_pose_1["pos"][0] - 0.11, marker_pose_1["pos"][1], marker_pose_1["pos"][2] + 0.08]
            lift_position_1 = [position_1[0], position_1[1], position_1[2] + 0.3]
            new_position_1 = [position_1[0], position_1[1] - 0.1, position_1[2]]

            # Move the arm to the object
            self.__move_arm_to_pose(position=position_1, orientation=fixed_tagQuat)

            if self.timer_count >= 3:
                self.__remove_collision_object_1(object_id=f"collision_obj_{1}")
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

                # Move robot to the right
                self.move_robot_right()

                # Create a new goal pose
                new_goal_pose = self.create_goal_pose(0.7005612031968964, 3.966066234090596, 0.0, 0.3352854202666504, 0.9421165994486116)
                self.publish_goal(new_goal_pose)

                # Destroy the timer after execution
                self.destroy_timer(self.timer)

        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {self.source_frame_1} to {self.target_frame}: {ex}")

    def move_robot_right(self):
        """Moves the robot to the right (positive Y direction) with velocity 0.5 for 7 seconds."""
        msg = Twist()
        msg.linear.y = 0.5  # Move in the positive Y direction (right)
        msg.angular.z = 0.0  # No rotation
        self.get_logger().info("Moving robot to the right for 7 seconds.")
        self.publisher_.publish(msg)
        time.sleep(7)
        msg.linear.y = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Robot stopped.")

    def __add_collision_box(self, object_id: str, position: list, orientation: list, dimensions: list):
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")

    def __add_collision_object_1(self, object_id: str, position: list, orientation: list, dimensions: list):
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")

    def __add_collision_object_2(self, object_id: str, position: list, orientation: list, dimensions: list):
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")

    def __add_collision_object_3(self, object_id: str, position: list, orientation: list, dimensions: list):
        self.__moveit2.add_collision_box(
            id=object_id,
            position=position,
            quat_xyzw=orientation,
            size=dimensions
        )
        self.get_logger().info(f"Collision box added with ID: {object_id}")

    def __move_arm_to_pose(self, position: list, orientation: list):
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

    def __open_gripper(self):
        self.__gripper_interface.open()
        self.__gripper_interface.wait_until_executed()

    def __partial_close_gripper(self):
        partial_close_position = 0.03
        self.__gripper_interface.move_to_position(partial_close_position)
        self.__gripper_interface.wait_until_executed()

    def __remove_collision_object_1(self, object_id: str):
        self.__moveit2.remove_collision_object(id=object_id)

    def handle_pick_and_place_request(self, request, response):
        self.get_logger().info(f"Received request to add collision object for marker ID: {request.marker_id}")
        if self.move:
            self.get_logger().info("Successfully movement planned object grasp and moved.")
            response.success = True
        else:
            self.get_logger().info("Collision box not added yet.")
            response.success = False
        return response

    def euler_from_quaternion(self, quaternion):
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
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        return roll_deg, pitch_deg, yaw_deg

    def quaternion_from_euler(self, euler_angles):
        roll, pitch, yaw = euler_angles
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy
        return [q[1], q[2], q[3], q[0]]

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    pick_and_place_object_node = Pickandplaceobject()
    executor.add_node(pick_and_place_object_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        pick_and_place_object_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()