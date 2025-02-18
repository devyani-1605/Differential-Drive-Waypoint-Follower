
# Gazebo Simulation Of Differential Drive Robot Controller with RPM Computation and Waypoint Navigation


This project implements a **ROS 2-based differential drive robot controller** that computes RPM for each wheel based on velocity commands and a **Python script for waypoint-based navigation** using odometry and PID control. The system is tested in **Gazebo**.




## Prerequisites

- ROS 2 (Foxy, Galactic, or Humble recommended) - Install ROS 2 from ROS official website

- Python 3.8+

- Gazebo


## Features

✅ **ROS 2 C++ Node** for differential drive control  
✅  **Gazebo Simulation Integration** for testing RPM control  
✅ **Python Script for Waypoint Navigation** using PID  
✅ **Configurable Parameters** for tuning  
✅ **ROS 2-based Topic Communication**  
## Run Locally

```bash
  mkdir ros2_ws/src
  cd ros2_ws/src
  git clone https://github.com/devyani-1605/Differential-Drive-Waypoint-Follower.git
```

## Build package

```bash
  cd ..
  colcon build 
  source install/setup.bash
```
## Part 1: Differential Drive Controller (C++)

Node Functionality

- Subscribes to /cmd_vel (velocity command)
- Computes wheel velocities and converts them to RPM
- Publishes RPM values to motor controllers
- Provides dynamically configurable parameters

Topics

| Topic Name        | Message Type            | Role                           |
|------------------|----------------------|------------------------------|
| `/cmd_vel`      | `geometry_msgs/Twist` | Input: Robot velocity command |
| `/left_wheel_rpm`  | `std_msgs/Float64`    | Output: Left wheel RPM        |
| `/right_wheel_rpm` | `std_msgs/Float64`    | Output: Right wheel RPM       |

ROS 2 Parameters

| Parameter Name | Type  | Description                     |
|---------------|------|---------------------------------|
| `wheelbase`  | double | Distance between wheels (m)    |
| `wheel_radius` | double | Radius of each wheel (m)     |
| `max_rpm`   | double | Max allowed RPM for safety    |

Testing

- Run these commands in different terminals sequentially:

```bash
  ros2 launch diff_drive_robot robot_gazebo.launch.py
```

```bash
  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```
```bash
  ros2 topic echo /left_wheel_rpm
  ros2 topic echo /right_wheel_rpm

```
## Part 2: Waypoint Navigation (Python)

Script Functionality

- Subscribes to odometry data (/odom)
- Uses a PID controller to adjust linear & angular velocity
- Navigates to two waypoints sequentially
- Publishes velocity commands to /cmd_vel
- Stops the robot after reaching both waypoints

Topics

| Topic Name | Message Type          | Role                     |
|------------|----------------------|--------------------------|
| `/odom`    | `nav_msgs/Odometry`  | Input: Current position  |
| `/cmd_vel` | `geometry_msgs/Twist` | Output: Velocity command |

ROS 2 Parameters

| Parameter | Type  | Description                |
|-----------|------|----------------------------|
| `waypoint_1_x` | double | X-coordinate of the first waypoint |
| `waypoint_1_y` | double | Y-coordinate of the first waypoint |
| `waypoint_2_x` | double | X-coordinate of the second waypoint |
| `waypoint_2_y` | double | Y-coordinate of the second waypoint |
| `kp` | double | Proportional gain for PID |
| `ki` | double | Integral gain for PID |
| `kd` | double | Derivative gain for PID |

Navigation 

```bash
  ros2 launch diff_drive_robot robot_gazebo.launch.py
```
```bash
  ros2 run differential_drive_controller waypoint_navigation.py --ros-args \
  -p waypoint_1_x:=2.0 -p waypoint_1_y:=1.0 -p waypoint_2_x:=4.0 -p waypoint_2_y:=3.0

```







## License

MIT License

