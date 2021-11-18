# MOMA navigation v1.0 : laser-scan + static map

### 0 - Launch Robot Interfaces

// old method launching locobot description

```ros2 launch locobot_description locobot_description.launch.py use_sim_time:=False gui:=False use_rviz:=False```

// new method using kobuki description (modified repo is needed)

```ros2 launch kobuki_description kobuki_description.launch.py use_sim_time:=False gui:=False use_rviz:=False```

// Launch the arm and camera motor controller

```cd ros2_ws/src/locobot_control/```

```ros2 run locobot_control locobot_control_node /dev/ttyUSB1 1000000```

// Check the current TF between robot footprint

```ros2 run tf2_ros tf2_echo base_footprint camera_link```

// Launch the camera node

```ros2 launch realsense2_camera devlab02_rs_launch.py```

// Launch the depth to laser scan node

```ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args --remap depth:=/devlab02_camera/depth/image_rect_raw --remap depth_camera_info:=/devlab02_camera/depth/camera_info```

// Launch the base controller

```ros2 launch  kobuki_node kobuki_node-composed-launch.py```

// Launch the teleoperation node

```ros2 run kobuki_keyop kobuki_keyop_node --ros-args --remap cmd_vel:=/commands/velocity --remap motor_power:=/commands/motor_power```

### 1- Launch Navigation2
```ros2 launch nav2_bringup navigation_launch_kobuki.py use_sim_time:=False```
2- Launch SLAM
```ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False```
// Display the current tf tree
```ros2 run tf2_tools view_frames.py```
3- Use Static Map
```ros2 launch kobuki_node kobuki_launch.py```
