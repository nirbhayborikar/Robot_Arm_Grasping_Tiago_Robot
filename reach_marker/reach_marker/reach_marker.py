import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from pymoveit2 import MoveIt2
from pymoveit2.robots import tiago as robot
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ReachMarkerNode(Node):
    def __init__(self):
        super().__init__("reach_marker")
        self.get_logger().info("Reach_Marker_Node is began")
        self.__callback_group = ReentrantCallbackGroup()

        # Parameters for arm movement
        self.__synchronous = True
        self.__cartesian = False
        self.__cartesian_max_step = 0.0025
        self.__cartesian_fraction_threshold = 0.0

        # Initialize MoveIt2
        self.__moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.__callback_group,
        )

        # MoveIt2 parameters
        self.__moveit2.planner_id = "RRTConnectkConfigDefault"
        self.__moveit2.max_velocity = 0.5
        self.__moveit2.max_acceleration = 0.5
        self.__moveit2.cartesian_avoid_collisions = False
        self.__moveit2.cartesian_jump_threshold = 0.0

        # TF listener for getting marker pose
        self.source_frame = "tag36h11:3"  # The marker's frame
        self.target_frame = "base_footprint"  # The base frame of the robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.markerPose = None
        self.goal_reached = False  # Flag to check if the goal is reached

        # Timer to get marker pose and move the arm every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # If the goal has already been reached, stop the task

        
        if self.goal_reached:
            self.get_logger().info("Goal already reached. No further movement will be made.")
            return

        try:
            # Get the transformation between the marker and the robot base
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )

            # Update the marker pose
            self.markerPose = transform

            # Extract orientation and position from the marker pose
            '''Here the the marker pose is extracted  which received from transform'''
            markerQuat = (
                self.markerPose.transform.rotation.x,
                self.markerPose.transform.rotation.y,
                self.markerPose.transform.rotation.z,
                self.markerPose.transform.rotation.w,
            )
            position = [
                self.markerPose.transform.translation.x - 0.224,  # Adjust the position if necessary
                self.markerPose.transform.translation.y,
                self.markerPose.transform.translation.z,
            ]
            self.get_logger().info(f"Marker coordinate received position: {position} , orientatio: {markerQuat}")
            # Move the arm to the desired position
            self.get_logger().info("Moving arm to pose ")
            self.__move_arm_to_pose(position=position, orientation=markerQuat)

        except TransformException as ex:
            self.get_logger().info(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")

    def __move_arm_to_pose(self, position: list[float, float, float], orientation: list[float, float, float, float]):
        """
        Move the arm to a given pose relative to the base footprint.

        Args:
        - position: 3D position (x, y, z) relative to the base footprint
        - orientation: quaternion orientation (x, y, z, w) relative to the base footprint
        """
        self.__moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            cartesian=self.__cartesian,
            cartesian_max_step=self.__cartesian_max_step,
            cartesian_fraction_threshold=self.__cartesian_fraction_threshold,
        )

        if self.__synchronous:
            self.__moveit2.wait_until_executed()
            self.get_logger().info("Successfully reached the desired position.")
            self.goal_reached = True  # Set the flag to stop further movements (This will stop publishing goal and robot hand movement stop)

def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor() # it used for running multi task concurrently
        reach_marker_node = ReachMarkerNode()
        executor.add_node(reach_marker_node) # on multithread executor I added the reach_marker_node

        try:
            executor.spin()
        finally:
            executor.shutdown()
            reach_marker_node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()