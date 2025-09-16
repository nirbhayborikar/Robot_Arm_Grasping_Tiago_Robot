import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from pymoveit2 import MoveIt2
from pymoveit2.robots import tiago as robot
from tutorial_interfaces.srv import Addcollision  # Ensure you have this service defined

class AddCollisionObjectService(Node):
    def __init__(self):
        super().__init__("add_collision_object_srv")
        # TF Listener to track marker pose
        self.source_frame = "tag36h11:1"  # Marker frame
        self.marker_id = 1
        self.target_frame = "base_link"  # Robot base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pose = None  # To store the latest marker pose
        self.goal_reached = False
        # Create service to handle requests
        self.service = self.create_service(Addcollision, 'add_collision_object', self.handle_add_collision_request)
        self.box_added = False  # Flag to track if box is added

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

        # Timer to update marker pose and manage collision objects
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer for periodic updates

        self.get_logger().info("AddCollisionObjectService initialized.")

    def timer_callback(self):
        """Periodically updates marker pose and performs collision object tasks.
        
        Here we use transformation or say tf to get marker pose and based on that 
         __add_collision_object function is called by passing required args """
        if self.goal_reached:
            self.get_logger().info("Goal already reached. No further updates.")
            return

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

            # Extract pose information
            pos_x = self.marker_pose.transform.translation.x
            pos_y = self.marker_pose.transform.translation.y
            pos_z = self.marker_pose.transform.translation.z
            rot_x = self.marker_pose.transform.rotation.x
            rot_y = self.marker_pose.transform.rotation.y
            rot_z = self.marker_pose.transform.rotation.z
            rot_w = self.marker_pose.transform.rotation.w

            # Log the pose details
            self.get_logger().info("Marker Pose Updated:")
            self.get_logger().info(f"Position: ({pos_x}, {pos_y}, {pos_z})")
            self.get_logger().info(f"Orientation: ({rot_x}, {rot_y}, {rot_z}, {rot_w})")

            # Compare with the last pose
            current_pose = (pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w)

            # the loop is used to puvlish box fo one time on the detected marker_pose reference
            if self.last_pose is None or current_pose != self.last_pose:
                # Pose has changed, add a new collision box
#0-16
                self.__add_collision_box(
                    object_id=f"collision_obj_{self.marker_id}",
                    position=[pos_x + 0.21, pos_y, pos_z  + 0.030],
                    orientation=[rot_x, rot_y, rot_z, rot_w],
                    dimensions=[0.5, 0.5, 0.1]
                )
                self.box_added = True  # Flag set to True when box is added
                self.last_pose = current_pose  # Update the last pose

        except TransformException as ex:
            self.get_logger().info(f"Could not transform {self.source_frame} to {self.target_frame}: {ex}")

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

    def handle_add_collision_request(self, request, response):
        """Handles requests to add a collision object.
        The client command will log the message of the service
        it takes marker_id as input
        and return response
        client can be called by (ros2_ws collision_service client)"""
        self.get_logger().info(f"Received request to add collision object for marker ID: {request.marker_id}")

        # Wait for the command by client to trigger collision box addition
        if self.box_added:
            self.get_logger().info("Successfully added collision object.")
            response.success = True
        else:
            self.get_logger().info("Collision box not added yet.")
            response.success = False

        return response


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    # Initialize the AddCollisionObjectService node
    add_collision_object_service = AddCollisionObjectService()
    executor.add_node(add_collision_object_service)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        add_collision_object_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
