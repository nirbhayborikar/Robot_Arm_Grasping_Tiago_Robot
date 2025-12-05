# TIAGO Robot Navigation and Manipulation Using ROS2, PyMoveIt2, and Docker  
**ROS2 • PyMoveIt2 • Manipulation Planning • SLAM • AprilTags • Gazebo Simulation**

This repository contains a complete framework for **TIAGO robot navigation and manipulation** using ROS2, PyMoveIt2, TF transformations, AprilTag perception, and Docker-based deployment.  
The project demonstrates autonomous navigation, object detection, collision-aware motion planning, and pick-and-place tasks in Gazebo and RViz.

📄 **Full Technical Report (PDF):**  
[TIAGO_ROS2_Manipulation_Report.pdf](https://github.com/nirbhayborikar/Robot_Arm_Grasping_Tiago_Robot/blob/main/TIAGO_ROS2_Manipulation_Report.pdf)

---

## 1. Overview

This project focuses on developing a robust algorithm for the **TIAGO robot arm** to automatically reach, grasp, and manipulate objects in a simulated environment.  
Using **PyMoveIt2**, the robot computes collision-free trajectories based on real-time TF transformations obtained from detected AprilTags.

### Key Capabilities

- Autonomous navigation using **ROS2 Navigation Stack**, **SLAM**, and **AMCL**  
- Accurate object localization using **AprilTag detection + TF transforms**  
- PyMoveIt2-based **motion planning, grasping, and manipulation**  
- Multi-threaded architecture for real-time perception + planning + execution  
- Fully containerized ROS2 workspace using **Docker**  
- Demonstrations of reaching targets, cleaning a table, pick-and-place, and marker tracking

---

## 2. Features

### **2.1 Robot Navigation & Localization**
- SLAM-based mapping in Gazebo  
- AMCL for localization  
- TF-based frame management  
- Goal navigation using ROS2 action servers  

### **2.2 AprilTag-Based Perception**
- Integrated **ros2_apriltag** package  
- Real-time pose extraction of AprilTag markers  
- Tags used to identify objects and navigation targets  
- TF broadcasting for all detected tags  

### **2.3 Manipulation with PyMoveIt2**
- Precise, collision-aware arm motion planning  
- Dynamic collision objects added to MoveIt planning scene  
- End-effector pose generation from AprilTag detection  
- Gripper control with pre-grasp and post-grasp strategies  

### **2.4 Pick-and-Place System**
- Multi-stage grasping pipeline  
- Pose estimation → pre-grasp → grasp → lift → place  
- Obstacle-aware trajectories  
- Modular ROS2 services and interfaces  

### **2.5 Simulation and Visualization**
- Gazebo: full 3D environment with realistic physics  
- RViz2:  
  - TF frames  
  - AprilTags  
  - Planned trajectories  
  - SLAM map  
  - Collision objects  

### **2.6 Software Architecture**
- MultiThreadedExecutor for concurrent:  
  - TF-listening  
  - Motion planning  
  - Navigation feedback  
- Python controllers integrated with PyMoveIt2  
- Dockerized ROS2 Humble/Foxy environment for reproducible builds  

---

## 3. Repository Structure

Below is a description of the main folders for quick navigation.

---

### **📁 docker/**
Contains Docker workspace configuration.  
- The primary file: **`tiago_reach_marker.yml`**  
  - Used to execute a reach-marker behavior with TIAGO’s arm  
  - Integrates camera detection + PyMoveIt2 planning  

Note: Ensure the **ros2_apriltag** package is running for all TF-based object detections.

---

### **📁 41-pick-and-place-object/**
Contains ROS2 packages for full pick-and-place pipeline:

1. **Main task execution**  
2. **ROS2 interface** — publishes/receives commands  
3. **Service folder** — returns feedback on pick-and-place request  

---

### **📁 43-clean-the-table/**
Implements a complete table-cleaning scenario.

- **initial_to_goal/**  
  Moves TIAGO from its initial position to the table where the object is located  
- **clean_the_table/**  
  Detect object → Reach → Pick → Move → Place operation (pick-and-place)

---

### **📁 add_collision_object/**
Contains a ROS2 service to add collision objects to the planning scene.

**Purpose:**  
Collision objects improve safety and motion planning reliability.  
They represent boxes, cylinders, meshes, etc., used by MoveIt to avoid obstacles in the environment (Gazebo or RViz).

---

### **📁 reach_marker/**
Contains a demonstration script:  
- Move TIAGO’s arm to an AprilTag marker pose  
- Uses TF transformation + PyMoveIt2 trajectory generation  

---

### **📁 videos/**
Contains demonstration videos including:

- Navigation to a goal using **AMCL**  
- Reaching an AprilTag marker  
- Pick-and-place execution  
- Full clean-the-table behavior  

---

## 4. Technical Details

### **Development Environment**
- Linux  
- ROS2 (Foxy / Humble)  
- Gazebo simulation  
- RViz2 visualization  
- Docker + Docker Compose  

### **Perception & Localization**
- SLAM for mapping  
- AMCL for pose estimation  
- AprilTag detection for object localization  
- TF2 for transformation propagation  

### **Manipulation Stack**
- PyMoveIt2 for motion planning  
- Collision object insertion  
- Multi-stage grasping controllers  
- Waypoint navigation and pre-grasp approach generation  

### **Concurrency**
- ROS2 MultiThreadedExecutor for parallel callback execution  
- Enables real-time sensor processing during motion execution  

---

## 5. Challenges and Solutions
- **Grasp accuracy:** improved by tuning gripper parameters and adding pre-grasp poses  
- **Motion planning failures:** solved via refined collision modeling and improved start states  
- **Object slipping:** addressed with adjusted gripper force and contact simulation  
- **Marker pose noise:** smoothed via TF filtering and pose averaging  

---

## 6. Full Report

A detailed explanation of:  
- algorithms  
- architecture  
- math  
- execution pipeline  
- debugging strategies  
- references  

…is provided in the full PDF report.

📄 **PDF:**  
[TIAGO_ROS2_Manipulation_Report.pdf](https://github.com/nirbhayborikar/Robot_Arm_Grasping_Tiago_Robot/blob/main/TIAGO_ROS2_Manipulation_Report.pdf)

---

## 7. Author

**Nirbhay Borikar**  
Portfolio: https://nirbhayborikar.github.io  
GitHub: https://github.com/nirbhayborikar  

---
