# robotino4-ros2

This repository contains six ROS  packages related to the Robotino robot.

## Included Packages:

### 1. robotino_description

This package contains the URDF model of the Robotino robot. It also includes a launch file to visualize the robot in RViz.

#### Usage Instructions:

To launch the robot model in RViz, execute the following command:

```bash
ros2 launch robotino_description display.launch.xml
```

### 2. robotino_bringup 

This package contains two launch files to spawn the Robotino robot in Gazebo with ros_planar_move plugin and visualize it in RViz. One of this launch file is for spawn the robot in an empty Gazebo world. An the other one spwan the robot in a turtlebot3 world.

#### Usage Instructions:

To launch the robot in a empty Gazebo world and visualize it in RViz, execute one of the two following commands:

```bash
rros2 launch robotino_bringup robotino_gazebo_rviz.launch.xml
```

```bash
ros2 launch robotino_bringup robotino_gazebo_rviz.launch.py
```

To launch the robot in a Gazebo world and visualize it in RViz:

```bash
ros2 launch robotino_bringup robotino_world.xml
``` 

To control the robot's wheels:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

### 3. cartographer_slam & map_server 

The **cartographer_slam** package facilitates the creation of a map for the robot's surrounding environment, while the **map_server** package aids in visualizing the map generated by cartographer.

#### Usage Instructions:

To launch cartographer, execute the folowing command:

```bash
ros2 launch cartographer_slam cartographer.launch.py
``` 

To create the map, move the robot around until the map appears correctly in RViz. To control the robot's movement, use the following command in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
``` 

Once the map is satisfactorily generated, execute the following command to save it:

```bash
cd ~/robotino4-ros2/src/cartographer_slam/config
ros2 run nav2_map_server map_saver_cli -f turtlebot_area
``` 

The saving command will generate two files:

- **tutlebot_area.pgm** image file is the map as an occupancy grid image.
- **turtlebot_area.yaml** file contains details about the resolution of the map.

To verify the successful generation of the map, launch the map server using

```bash
ros2 launch nav_map_server.launch.py
``` 

### 4. localization_server

The **localization_server** packages is designed to localize the robot, providing its precise position and orientation within the environment.

#### Usage Instructions:

Executed the following commands:

```bash
ros2 launch robotino_bringup robotino_world.xml
``` 

```bash
ros2 rviz2
``` 

```bash
ros2 launch localization_server localization.launch.py
``` 

In RViz, save the initial pose using the "2D Estimate Pose" tool located in the toolbar above.

### 5. path_planner

The **path_planner** packages move the robot from Point A to Point B, avoiding the obstacles along the map we have generated. It leverages the Dijkstra algorithm for path planning. 

#### Usage Instructions:

Executed the following commands:

1. Launch the robot simulation environment using:

```bash
ros2 launch robotino_bringup robotino_world.xml
``` 

2. Open RViz for visualization:

```bash
rviz2
``` 

3. Start the localization server:

```bash
ros2 launch localization_server localization.launch.py
``` 

In RViz, save the initial pose using the "2D Estimate Pose" tool located in the toolbar above.

4. Launch the path planner server:

```bash
ros2 launch path_planner_server pathplanner.launch.py
``` 

In RViz, put a gol pose with "2D Goal Pose" tool locatedd in the toolbar above. 

### 6. robotino_rest_node

This package contains two nodes for interacting with the Robotino robot via a REST API.

- **robotino_bumper**: This node reads the state of the robot's bumper via the REST API and publishes the data on the "bumper" topic. It also prints the data to the terminal.

- **robotino_omnidrive**: This node subscribes to the "cmd_vel" topic to receive velocity commands and sends them to the Robotino robot via the REST API.

#### Usage Instructions:

To run the `robotino_bumper` node, use the following command:

```bash
ros2 run robotino_rest_node robotino_bumper
```

To subscribe to the "bumper" topic and view bumper data, use the following command:

```bash
ros2 topic echo bumper
```

To run the `robotino_omnidrive` node, use the following command:
```bash
ros2 run robotino_rest_node robotino_omnidrive
```

To send velocity commands to the robotino via the "cmd_vel" topic, use the following command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

## Setting up Environment

To ensure everything functions correctly, add the following lines to your .bashrc file:

```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robotino4-ros2/src/
source ~/robotino4-ros2/install/setup.bash
```

In the .gazebo/models folder, it's necessary to include the turtlebot3_world folder in order to launch the robotino_world.xml.

## TODO:

- [X] Organize the URDF file (reintroduce xacros and separate the code as in the course).
- [ ] Inside the cartographer_slam include the map_server to check if the map is correct. 
- [ ] In the new packages(cartographer, localization and map) introduced in launch file the rviz and the launch tha spawn robotino in a turtlebot Gazebo world.
- [ ] Modify the launch file or the URDF file so that the robot meshes are exported correctly to Gazebo without needing to include the export GAZEBO_PATH in the bash file.
- [ ] Find a way to place the meshes that the world uses without having to put them in the .gazebo folder.
