## Group 7 RWA3 submission
# Running the Code
1. Please ensure the packages are setup as mentioned in RWA3 excluding the submitted package group 7.
2. Paste the package under the src folder in the workspace that is workspace_name/src.
3. Ensure that you "export TURTLEBOT3_MODEL=waffle " as well the package is sourced as "source install/setup.bash".
4. Please run "ros2 launch turtlebot3_gazebo maze.launch.py".
5. After the window has opened and the robot has spawned,Please run the following command "ros2 launch group7 test.launch.py"

The output  video is here (https://drive.google.com/file/d/1U3VVtPFYkcKQQu66TaybuIGqNnOGlkQF/view?usp=drive_link).


# Some additonal information:

* We had to correct the downward tilt of the logical camera manually which was not being recognized in the information it was outputting. This version works perfectly with the world set up in the assignment, if you go ahead and correct the camera node after submission the positions might be off due to our additional transform.
* Low real-time factor (less than 0.9) might disturb the turn trajectory, so ensure it is running with ~1 real time factor in Gazebo