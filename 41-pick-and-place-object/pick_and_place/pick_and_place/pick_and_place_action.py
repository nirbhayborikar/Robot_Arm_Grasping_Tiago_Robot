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

import numpy as np
import math



class Pickandplaceobject(Node):
    def __init__(self):
        super().__init__("pick_and_place_object")
        # TF Listener to track marker pose
        self.source_frame = "tag36h11:1"  # Marker frame
        self.marker_id = 1
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
        self.__moveit2.max_velocity = 0.5
        self.__moveit2.max_acceleration = 0.5
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

    def timer_callback(self):
        """Periodically updates marker pose and performs collision object tasks.
        
        Here we use transformation or say tf to get marker pose and based on that 
         __add_collision_object function is called by passing required args """
  

        try:
            # Check if transform is available
            if not self.tf_buffer.can_transform(self.target_frame, self.source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().info(f"Waiting for transform {self.source_frame} -> {self.target_frame}")
                return

            # Get the transformation between the marker and the robot base
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            self.marker_pose = transform
            self.tagPose = transform

            # Extract pose information
            pos_x = self.marker_pose.transform.translation.x
            pos_y = self.marker_pose.transform.translation.y
            pos_z = self.marker_pose.transform.translation.z
            rot_x = self.marker_pose.transform.rotation.x
            rot_y = self.marker_pose.transform.rotation.y
            rot_z = self.marker_pose.transform.rotation.z
            rot_w = self.marker_pose.transform.rotation.w


            self.timer_count += 1

            #tagQuatCollision = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w
            self.get_logger().info("Marker Pose Updated:")
            self.get_logger().info(f"Position: ({pos_x}, {pos_y}, {pos_z})")
            self.get_logger().info(f"Orientation: ({rot_x}, {rot_y}, {rot_z}, {rot_w})")

            self.__open_gripper()


            #Orientation data collection
            tagQuat = self.tagPose.transform.rotation.x,self.tagPose.transform.rotation.y,self.tagPose.transform.rotation.z,self.tagPose.transform.rotation.w

            #converting Quat to Euler
            tagEuler = self.euler_from_quaternion(tagQuat)
            #Fixing orientation
            fixed_tagEuler = (tagEuler[0]-90,tagEuler[1],tagEuler[2]-90)
            #converting fixed Euler to Quat
            fixed_tagQuat = self.quaternion_from_euler(fixed_tagEuler)


            #adding the boxes
            if self.box_added == False:
                self.__add_collision_box(object_id=f"collision_obj_{self.marker_id}",position=[pos_x + 0.21, pos_y, pos_z  - 0.067],orientation=[rot_x, rot_y, rot_z, rot_w],dimensions=[0.5, 0.5, 0.1])
                self.box_added = True
            else: 
                self.get_logger().info("adding box")

                    
            

            #position data collection
            position = [self.tagPose.transform.translation.x-0.11,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z+0.08]
            lift_position = [self.tagPose.transform.translation.x-0.11,self.tagPose.transform.translation.y,self.tagPose.transform.translation.z+0.08+0.3]
            new_position = [self.tagPose.transform.translation.x-0.11,self.tagPose.transform.translation.y-0.1,self.tagPose.transform.translation.z+0.08]

            #arm movement
            self.get_logger().info("Moving arm to pose")
            self.__move_arm_to_pose(position=position, orientation=fixed_tagQuat)
            if self.timer_count >= 3:
                
                # self.__remove_collision_object(object_id="box")
                
                self.__remove_collision_object(object_id="box")
                self.get_logger().info('BoxCollider removed')
                
                self.get_logger().info('Partially closing Gripper')
                #self.__close_gripper()
                self.__partial_close_gripper()
                time.sleep(3)
                self.get_logger().info('lifting  object')
                self.__moveit2.max_velocity = 0.2
                self.__moveit2.max_acceleration = 0.2
                self.__move_arm_to_pose(position=lift_position, orientation=fixed_tagQuat)
                time.sleep(3)
                self.get_logger().info('placing  object')
                self.__move_arm_to_pose(position=new_position, orientation=fixed_tagQuat)

                t = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rclpy.time.Time())

                self.wristPose = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                #self.get_logger().info(self.wristPose)



                self.get_logger().info('Destroying the timer after 3 executions.')
                self.destroy_timer(self.timer)
        except TransformException as ex:
            self.get_logger().info(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")
        

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
        partial_close_position = 0.0195 
        #partial_close_positions = 0.3  # Example values for partial closure
        self.__gripper_interface.move_to_position(partial_close_position)
        self.__gripper_interface.wait_until_executed()




    def __remove_collision_object(self, object_id:str):
        """
        Remove collision object based on the object id
        
        Args:
          object_id : Name of the object which has to be removed
        """
        self.__moveit2.remove_collision_object(id=object_id)



    

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