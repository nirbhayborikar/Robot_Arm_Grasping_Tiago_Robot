import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from tutorial_interfaces.srv import Addcollision
import time  # Correctly importing Python's built-in time module

class AddCollisionObjectClient(Node):
    def __init__(self):
        super().__init__('add_collision_object_client')
        self.client = self.create_client(Addcollision, 'add_collision_object')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            time.sleep(4)  # Use Python's time.sleep for delays

    def send_request(self, marker_id):
        # Create a request object
        request = Addcollision.Request()  # Fix the request creation here
        request.marker_id = marker_id  # Set marker_id in request

        # Call the service
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    # Callback to handle service response
    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully added collision object!")
            else:
                self.get_logger().info("Failed to add collision object.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    client = AddCollisionObjectClient()

    # Send request to the service with marker ID 1 (example)
    client.send_request(1)

    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
