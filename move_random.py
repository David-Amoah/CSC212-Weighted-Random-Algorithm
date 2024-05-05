#!/qcc/bin/env python3

#Authors: David Restrepo & David Amoah


'''
@mainpage Weighted Random Algorithm

This documentation provides an overview of the weighted random algorithm implemented in Python for a Roomba robot

##Introduction
The weighted random algorithm is designed to guide a Roomba robot through a
space while taking into account different weighted probabilities for each direction.
This approach allows the Roomba to make intelligent decisions based on
factors such as obstacle proximity, cleaning efficiency, and user preferences.

## Algorithm Overview

The algorithm works by assigning weights to each possible direction the Roomba can take (e.g., forward, backward, left, right). These weights are based on various factors, such as obstacle proximity, cleanliness of the area, and predefined preferences set by the user.

During navigation, the algorithm generates a random number within a predefined range and selects the direction corresponding to that number based on the assigned weights. This ensures that the Roomba makes decisions that are both random and weighted towards more favorable directions.

## Implementation with Python

The algorithm is implemented in Python using object-oriented programming principles. The Roomba class encapsulates the functionality for navigating the robot, while the weighted random algorithm is implemented as a method within this class.

The Python implementation provides flexibility for adjusting weights, defining custom criteria for direction selection, and integrating with other modules or sensors present in the Roomba robot.
'''

import rclpy   # imports the ros2 python client library
import math   # imports the math module
import time    # imports the time module
import os       #imports the python os module for interacting with the operating system
import random   #imports the random module for random number generation
from rclpy.node import Node #imports the node class from the rclpy.node module
from rclpy.qos import ReliabilityPolicy, QoSProfile #imports classes related to quality of service settings for ROS2
from geometry_msgs.msg import Twist, PoseStamped #imports message types from geometry_msgs package
from irobot_create_msgs.msg import HazardDetectionVector #custom ROS2 message for hazard detection
from nav2_simple_commander.robot_navigator import BasicNavigator #imports module for robot navigation
import tf_transformations #module for working with transformations in ROS



#Function to create a poseStamped message with position and orientation values
def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    """
       Creates a PoseStamped message with the specified position and orientation.

       This function creates a PoseStamped message with the given position and orientation
       parameters and sets the frame ID and timestamp in the header.

       Args:
           navigator (BasicNavigator): The instance of BasicNavigator used to get the current timestamp.
           position_x (float): The X-coordinate of the position.
           position_y (float): The Y-coordinate of the position.
           orientation_z (float): The Z-coordinate of the orientation in radians.

       Returns:
           geometry_msgs.msg.PoseStamped: The created PoseStamped message.

       Note:
           The function calculates the quaternion from Euler angles to set the orientation of the pose.
       """
    #Calculate the quaternion from Euler angles
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)

    #Creates a PoseStamped message
    pose = PoseStamped()

    #Set frame Id in the header of the poseStamped message
    pose.header.frame_id = 'map' #frame ID
    pose.header.stamp = navigator.get_clock().now().to_msg() #current timestamp

    #sets position of the pose
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0

    #sets orientation of the pose
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    #returns the created PoseStamped message
    return pose

#fucntion that checks if a map with a specified exists
def is_map_found(map_name):
    """
        Checks if a map file with the specified name exists.

        This function constructs the path to the YAML file based on the provided map name and
        checks if the file exists at that path. It prints a message indicating whether the map
        file is found or not.

        Args:
            map_name (str): The name of the map to be checked.

        Note:
            The function constructs the path to the YAML file by concatenating the provided map name
            with the directory path where the map files are stored.
        """
    # Construct the path to the YAML file based on the map name

    path = "/home/qcc/ros2_ws/src/py_pubsub/saved_maps/" + map_name + ".yaml"
    #checks if the file exists at a specified path and then prints a message
    if (os.path.exists(path)):
        print("FOUND")
        
        # while not nav.isTaskComplete():
        #     feedback = nav.getFeedback()
        
        # print(nav.getResult())

    else:
        print("NOT FOUND")

#function to check if a hazard is detected
def is_front_hazard_active(hazards): #Returns T/F if a hazard is detected
    """
        Checks if a hazard is detected in front.

        This function iterates through the detections in the given hazards object and checks
        if any hazard is detected in the front. It returns True if a hazard is detected,
        otherwise returns False.

        Args:
            hazards: A hazards object containing information about detected hazards.

        Returns:
            bool: True if a hazard is detected in front, False otherwise.

        Note:
            The function assumes that the hazards object has a 'detections' attribute, which is
            expected to be an iterable containing information about individual detections.

            The function uses the value of 'detection.type' to determine if a hazard is detected.
            A value of 0 represents no hazard, and any other value represents a hazard.

        Example:
            >>> hazards = get_front_hazards()
            >>> is_front_hazard_active(hazards)
            True
        """
    for detection in hazards.detections:
        #checks and returns True/False if a hazard is detected.
        # Uses 0 to represent no hazard and any other value to represent hazard
        if (detection.type != 0):
            return True
        return False

#Function to generate a randon waypoint with manual boundaries
def generateRandomWaypoint(): # SET MANUAL BOUNDARIES
    """
    Generates a random waypoint within specified boundaries.

    This function generates a random waypoint within manually set boundaries for the position
    (x, y) and rotation angle (z).

    Returns:
        Tuple[float, float, float]: A tuple containing the generated position (x, y)
        and rotation angle (z).

    Note:
        The boundaries for position_x, position_y, and rotation_z are manually set in this function.
    """
    position_x = random.uniform(-2.5, 1.0)
    position_y = random.uniform(-1.0, 0.15)
    rotation_z = random.uniform(0, 2.0 * math.pi) #random rotation angle

    #Returns the generated position
    return position_x, position_y, rotation_z

#defines a class for random movement
class MoveRandom(Node):
    """
    ROS 2 node for controlling a robot to move randomly.

    This class represents a ROS 2 node that controls a robot to move randomly in a given environment.
    It subscribes to hazard detection messages and navigates the robot to random waypoints while
    avoiding hazards.

    """

    def __init__(self):
        """
        Initializes the MoveRandom node.

        This method initializes the MoveRandom node, creates publishers and subscribers, and
        initializes parameters. It also sets up the initial state of the node.

        """

        super().__init__("move_random") #initializes the node

        #creates a publisher for sending velocity command
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

        #creates a subscriber for receiving hazard detection messages
        self.hazard_subscriber_ = self.create_subscription(
            HazardDetectionVector, "/hazard_detection", self.hazard_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #initializes a time to start
        """
        Initializes the time to start
        """
        self._start_time = 0

        #declares a parameter for the map name
        self.declare_parameter("map_name", rclpy.Parameter.Type.STRING)
        #get the parameter value
        self.map_name = self.get_parameter("map_name")
        self.get_logger().info("Move Random has been started") #message to indicate that the Move_Random has started
        self.generate_map() #generates the map

    #callback function for processing hazard detection messages
    def hazard_callback(self, hazards: HazardDetectionVector):
        """
        Callback function for processing hazard detection messages.

        This method is called whenever a hazard detection message is received. It checks if
        hazards are detected and triggers movement control accordingly.

        Args:
            hazards (HazardDetectionVector): The hazard detection messages received.
        """
        if (is_front_hazard_active(hazards)):
            self._start_time = time.perf_counter() #sets timer clock to current time
            self.move(True)
        else:
            self.move(False)

    #function for controlling the robot movement
    def move(self, hazard_detected):
        """
        Controls the robot movement.

        This method controls the movement of the robot based on whether hazards are detected
        and the current time.

        Args:
            hazard_detected (bool): True if hazards are detected, False otherwise.
        """
        cmd = Twist()
        nav = BasicNavigator()
        
        print("works")
        initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
        nav.setInitialPose(initial_pose)
        nav.waitUntilNav2Active()

        num_waypoints = 15 #variable for number of random waypoints
        waypoints = [] #list to store generated waypoints
        for _ in range(num_waypoints):
            #generates random waypoints and append to the list
            waypoint = generateRandomWaypoint()
            waypoints.append(create_pose_stamped(nav, *waypoint))

        if (hazard_detected or self._start_time + 1.5 > time.perf_counter()): #if hazard is detected and timer is started for 1.5 secs
                print("hazard")
                cmd.linear.x = 0.0 #sets linear velocity to 0
                cmd.angular.z = math.pi/2 #choose next grid around it randomly

        #move the robot to each generated random waypoint
        for waypoint in waypoints:
            nav.goToPose(waypoint) #navigates to the waypoint
            while not nav.isTaskComplete(): #wait until navigation tas is complete
                feedback = nav.getFeedback()

        print(nav.getResult()) #prints navigation result

            # cmd.linear.x = 0.25
            # cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd) #publish velocity command to control robot

    #calls generate_map function
    def generate_map(self):
        """
        Calls the generate_map function.

        This method calls the generate_map function to check if a map file with the specified
        name exists.
        """
        is_map_found(self.map_name.value) #checks if map is found

    # ros2 launch create3_lidar_slam sensors_launch.py
    # ros2 launch create3_lidar_slam slam_toolbox_launch.py
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # ros2 launch create3_lidar_slam rviz_launch.py

    #(command to save map)ros2 run nav2_map_server map_saver_cli -f my_map -f ~/ros2_ws/src/py_pubsub/saved_maps/my_map
        
    # 1. ros2 launch create3_lidar_slam sensors_launch.py
    # 2. ros2 launch create3_lidar_slam rviz_launch.py
    # 3. ros2 launch nav2_bringup localization_launch.py map:=/home/qcc/ros2_ws/src/py_pubsub/saved_maps/my_map.yaml use_sim_time:=false
    # 4. ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
    # 5. ros2 run py_pubsub move_random --ros-args -p map_name:=my_map

def main(args=None):
    """
       Entry point for the ROS 2 Python script.

       This function initializes the ROS 2 client library, creates an instance of a ROS 2 node,
       enters a loop to keep the node running, and finally shuts down the ROS 2 client library.

       Args:
           args (list): Command-line arguments passed to the script (default: None).

       Note:
           This function is typically used as the entry point for ROS 2 Python scripts.
           It initializes the ROS 2 client library, creates a node, and enters the event loop to
           handle ROS 2 messages and callbacks.
       """
    #intialize ros2 client library
    rclpy.init(args=args)

    #create an instance of moveRandom node
    node = MoveRandom()

    #enter a loop to keep the node running
    rclpy.spin(node)

    rclpy.shutdown() #shutdown ros2 client library

if __name__ == '__main__':
    """
       Entry point for executing the script as a standalone program.

       This block of code is executed only if the Python script is run directly as a standalone program.
       It provides a way to define a main function or perform specific tasks when the script is executed directly.
       """
    main()
