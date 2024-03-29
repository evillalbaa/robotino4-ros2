# robotino4-ros2

This repository contains three ROS 2 packages related to the Robotino robot.

## Included Packages:

### 1. robotino_bringup 

This package contains two launch files to spawn the Robotino robot in Gazebo with ros_planar_move plugin and visualize it in RViz. One of this launch file is for spawn the robot in an empty Gazebo world. An the other one spwan the robot in a turtlebot3 world.

#### Usage Instructions:

To launch the robot in a empty Gazebo world and visualize it in RViz, execute one of the two following commands:

```bash
ros2 launch robotino_bringup robotino_gazebo.launch.xml
```

```bash
ros2 launch robotino_bringup robotino_gazebo.launch.py
```

To launch the robot in a Gazebo world and visualize it in RViz:

```bash
ros2 launch robotino_bringup robotino_gazebo.launch.py
``` 

To control the robot's wheels:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10
```

### 2. robotino_description

This package contains the URDF model of the Robotino robot. It also includes a launch file to visualize the robot in RViz.

#### Usage Instructions:

To launch the robot model in RViz, execute the following command:

```bash
ros2 launch robotino_description display.launch.xml
```


### 3. robotino_rest_node

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
