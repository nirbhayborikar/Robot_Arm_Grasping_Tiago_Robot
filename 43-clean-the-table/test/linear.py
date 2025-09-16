import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State variables
        self.current_command = 0  # Tracks which command is being executed
        self.command_start_time = self.get_clock().now().seconds_nanoseconds()[0]  # Start time of the current command
        self.command_duration = 0  # Duration of the current command
        self.commands = [
            {"linear_x": 0.0, "angular_z": -0.707, "duration": 15.3},  # Command 1: 95 iterations at 10 Hz
            {"linear_x": 0.8, "angular_z": 0.0, "duration": 5.2},     # Command 2: 30 iterations at 10 Hz
            {"linear_x": 0.0, "angular_z": 0.707, "duration": 7.6},   # Command 3: 76 iterations at 10 Hz
            {"linear_x": 0.5, "angular_z": 0.0, "duration": 3.1},      # Command 4: 31 iterations at 10 Hz
        ]

    def timer_callback(self):
        # Get the current time
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Check if the current command has completed
        if current_time - self.command_start_time >= self.command_duration:
            # Move to the next command
            self.current_command += 1

            # Stop if all commands are completed
            if self.current_command >= len(self.commands):
                self.get_logger().info("All commands completed. Stopping the robot.")
                self.stop_robot()
                self.timer.cancel()  # Stop the timer
                return

            # Start the next command
            self.command_start_time = current_time
            self.command_duration = self.commands[self.current_command]["duration"]
            self.get_logger().info(f"Starting command {self.current_command + 1}")

        # Publish the current command
        msg = Twist()
        msg.linear.x = self.commands[self.current_command]["linear_x"]
        msg.angular.z = self.commands[self.current_command]["angular_z"]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear_x={msg.linear.x}, angular_z={msg.angular.z}')

    def stop_robot(self):
        # Stop the robot by publishing a zero velocity command
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Stopping the robot.")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()