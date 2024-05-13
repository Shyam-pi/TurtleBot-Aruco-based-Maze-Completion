# Turtlebot Aruco based Maze Navigation

![image](https://github.com/Shyam-pi/TurtleBot-Aruco-based-Maze-Completion/assets/57116285/937147e1-9fdd-48d8-95d2-5818cec30aa8)

**Note**: This project was done in collaboration with <a href='https://github.com/theunknowninfinite'>Jayasuriya Suresh</a> and <a href='https://github.com/SakshamV'>Saksham Verma</a> for ENPM809Y at the University of Maryland, College Park

## Overview
This project implements a ROS package for navigating a Turtlebot through a maze environment using Aruco markers for localization and object detection.

## Objectives Achieved
- Successfully developed a ROS package to control the Turtlebot's movement through the maze.
- Implemented Aruco marker detection for accurate localization and navigation.
- Integrated logical cameras for detecting and reporting objects within the maze environment.
- Ensured comprehensive documentation of code using Doxygen standards.
- Submitted the completed package adhering to all submission guidelines and deadlines.

## Setup and Installation
1. Ensure ROS and OpenCV dependencies are installed.
2. Clone the repository and navigate to the project directory.
3. Follow the setup instructions provided in the `rwa3_setup.sh` script or manually create a ROS workspace.
4. Create a new package within the workspace with the necessary dependencies.
5. Adjust system configurations as needed to work with the new workspace.

## Task Description
### 1. Simulation Start
- Launch the simulation environment using the provided launch file (`turtlebot3_gazebo maze.launch.py`).
- The environment includes a Turtlebot equipped with an RGB camera and logical cameras for object detection.

### 2. Node Implementation
- Developed ROS nodes to control the Turtlebot's movement through the maze.
- Utilized the RGB camera to detect Aruco markers for localization and navigation.
- Implemented logic to navigate the Turtlebot based on Aruco marker detections and associated parameters.
- Reported detected object poses within the maze environment.

## Documentation
- Comprehensive documentation of all code, methods, and attributes using Doxygen standards.
- Detailed comments explaining the functionality of each component and method.

## Running the Code
1. Please ensure the packages are setup as mentioned in RWA3 excluding the submitted package group 7.
2. Paste the package under the src folder in the workspace that is workspace_name/src.
3. Ensure that you ```export TURTLEBOT3_MODEL=waffle``` as well the package is sourced as ```source install/setup.bash```.
4. Please run ```ros2 launch turtlebot3_gazebo maze.launch.py```.
5. After the window has opened and the robot has spawned, please run the following command

```ros2 launch group7 test.launch.py```

The output  video is here (https://drive.google.com/file/d/1U3VVtPFYkcKQQu66TaybuIGqNnOGlkQF/view?usp=drive_link).


## Some additonal information:

* We had to correct the downward tilt of the logical camera manually which was not being recognized in the information it was outputting. This version works perfectly with the world set up in the assignment, if you go ahead and correct the camera node after submission the positions might be off due to our additional transform.
* Low real-time factor (less than 0.9) might disturb the turn trajectory, so ensure it is running with ~1 real time factor in Gazebo
