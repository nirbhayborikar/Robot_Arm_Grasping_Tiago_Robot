---------------------------------------
**TIAGO Robot Navigation, and Manipulation with ROS2 and Docker**

Developed multithread motion and manipulation planning in ROS 2 using PyMoveIt2. Performed SLAM, TF-
based localization, and target identification in Gazebo/Rviz; containerized workspace via Docker.
TIAGO Robot from PAL Robotics used for testing medical assistant feature. 
----------------------------------------

---------
**Complete Pdf: TIAGO_ROS2_Manipulation_Report.pdf** (It has detailed overview of algorithms and logic that used in this project)
---------

-------------------------------------------------------------------------
Folder: docker

It has various files.
The file that can be used ar tiago_reach_marker.yml, for making a movement using camera the robot arm.
......................

Furthermore, the ros apriltag package need to be integrate while using all this ros2 command as all object are attached to AprilTag marker for detecting in 3d environment.
..........................................................................

Folder; 41-pick-and-place-object

This folder contain 3 more folders. First contain main code for task execution, second 
 contain the interface for send messages, and last one is service folder on request it gove feedbacks.
........................................................................... 
Folder: 43-clean-the-table
 
It has two folder first it goes from initial position to goal position where the object is there that need to be picked.( Folder initial_to_goal)
 
Another folder is clean_the_table which had main files to detect object , reach to object pick and move to different table or place to put. (Pick and place) Folder: clean_the_table
...............................................................................

Folder: add_collision_object

It has service file to add collision object. In Gazebo and Rviz using pymoveit2, a Collision Object is used to define obstacles or environmental constraints in a robotic workspace. This is crucial for motion planning, ensuring the robot avoids collisions while executing movements. A Collision Object is typically represented as a geometric shape (e.g., box, sphere, cylinder, or mesh) with a defined position and orientation in the world. These objects help in path planning by informing the motion planner about restricted regions in the environment. 

-------------------------------------------------------------------------------
Folder: reach_marker

This folder contain file with a demo to reach AprilTag using tf transformation with PyMoveIt2 motion planner.
...............................................................................

Folder: videos

It has some video of TIAGO Robot in action like navigating to goal position using AMCL Adaptive Monte Carlo Localization.

Reach the marker video using PyMoveIt2

Pick and place object, again using Tf transformation and PyMoveIt2 motion planner.
--------------------------------------------------------------------------------
