





def moving_robot(self, distance: float, speed: float):
        """
        Moves the robot forward or backward for a given distance.
        :param distance: Distance to move (positive = forward, negative = backward) [meters]
        :param speed: Speed (positive = forward, negative = backward) [m/s]
        """
        while self.robot_halted == False:
            self.get_logger().info(f"waiting for robot to halt")
            if self.robot_halted:
                break

            
        move_cmd = Twist()
        move_cmd.linear.x = speed  # Speed can be positive or negative
        move_cmd.angular.z = 0.0

        start_time = time.time()
        distance_travelled = 0.0

        while abs(distance_travelled) < abs(distance):
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            elapsed_time = current_time - start_time
            distance_travelled = elapsed_time * speed  # Accounts for direction

            time.sleep(0.05)  # Optional: Smooth control loop

        # Stop the robot
        move_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Moved {distance} meters.")

    def emergency_brake(self):
        """
        Immediately stops the robot by publishing zero velocity.
        """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(move_cmd)  # Send stop command
        self.get_logger().info("Emergency stop activated!")

    

    
    def rotate_robo(self, angle_degrees: float, angular_speed: float):
        """
        Rotates the robot by a given angle.
        :param angle_degrees: Angle to rotate (positive = counterclockwise, negative = clockwise) [degrees]
        :param angular_speed: Rotation speed [radians/sec]

        """
        while self.robot_halted == False:
            self.get_logger().info(f"waiting for robot to halt")
            if self.robot_halted:
                break

        move_cmd = Twist()
        angular_speed = -abs(angular_speed) if angle_degrees < 0 else abs(angular_speed)  # Ensure correct direction
        move_cmd.angular.z = angular_speed

        angle_radians = angle_degrees * (3.1415926535 / 180.0)
        start_time = time.time()
        angle_travelled = 0.0

        while abs(angle_travelled) < abs(angle_radians):
            self.cmd_vel_pub.publish(move_cmd)
            current_time = time.time()
            angle_travelled = abs((current_time - start_time) * angular_speed)
            time.sleep(0.05)  # Ensures smooth control loop

        # Stop rotation
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        self.get_logger().info(f"Rotated {angle_degrees} degrees.")