# CSC212-Weighted-Random-Algorithm
This is an implementation of a weighed random algorithm in python for a create3 iRobot Roomba.

This is a research project which uses a Weighted Random Algorithm. It works by assigning numeric values to different objects and randomly picks one. Objects with higher 
values are more likely to be chosen. 

## Hardware
- Create 3 Roomba
- Raspberrypi 4
- Lidar sensor

## Software
- Ubuntu 22.04
- ROS 2
- NAV 2

## User Manual 
- Before using weighted random algortithm, ensure that your robot is powered on and ready for operation.  
- Ensure that robot is equipped with required sensors responsible for navigation.  (Lidar)
- Download Ros2 on a raspberry pi running on Ubuntu 22.04. [Link Here](https://docs.ros.org/en/crystal/Installation/Linux-Install-Binary.html)
- Install Nav 2 by running this command in the terminal.
  
      $ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

- Install slam toolbox

      $ sudo apt install ros-humble-slam-toolbox

- With the SLAM Toolbox installed, users can generate a detailed map of their environment, providing an accurate representation of the room layout.

- These steps enable to map out the room and manually control the robot via keyboard input, facilitating user-driven operation.
   >
      $ ros2 launch create3_lidar_slam sensors_launch.py  
      $ ros2 launch create3_lidar_slam slam_toolbox_launch.py  
      $ ros2 run teleop_twist_keyboard teleop_twist_keyboard
      $ ros2 launch create3_lidar_slam rviz_launch.py
   >

- Run this command in order to save the map

      $ ros2 run nav2_map_server map_saver_cli -f my_map -f

### Running the code
#
1. Start the slam tool box

        $ ros2 launch create3_lidar_slam slam_toolbox_launch.py
   
2. Start your lidar sensors
   
        $ ros2 launch create3_lidar_slam sensors_launch.py

3. Launch your rviz

        $ ros2 launch create3_lidar_slam rviz_launch.py
  
4. Set up and configure the localization components with this command

        $ ros2 launch nav2_bringup localization_launch.py
  
5. Execute the NAV2 launch file to activate navigation services.
   
        $ ros2 launch nav2_bringup navigation_launch.py

6. Run Move Random program

        $ ros2 run py_pubsub move_random --ros-args -p map_name:=my_map

# [Official documentation](https://newdocu.netlify.app/)
