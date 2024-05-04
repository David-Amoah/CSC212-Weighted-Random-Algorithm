#!/qcc/bin/env python3
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
    for detection in hazards.detections:
        #checks and returns True/False if a hazard is detected.
        # Uses 0 to represent no hazard and any other value to represent hazard
        if (detection.type != 0):
            return True
        return False

#Function to generate a randon waypoint with manual boundaries
def generateRandomWaypoint(): # SET MANUAL BOUNDARIES
    position_x = random.uniform(-2.5, 1.0)
    position_y = random.uniform(-1.0, 0.15)
    rotation_z = random.uniform(0, 2.0 * math.pi) #random rotation angle

    #Returns the generated position
    return position_x, position_y, rotation_z

#defines a class for random movement
class MoveRandom(Node):

    def __init__(self):

        super().__init__("move_random") #initializes the node

        #creates a publisher for sending velocity command
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

        #creates a subscriber for receiving hazard detection messages
        self.hazard_subscriber_ = self.create_subscription(
            HazardDetectionVector, "/hazard_detection", self.hazard_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        #initializes a time to start
        self._start_time = 0

        #declares a parameter for the map name
        self.declare_parameter("map_name", rclpy.Parameter.Type.STRING)
        #get the parameter value
        self.map_name = self.get_parameter("map_name")
        self.get_logger().info("Move Random has been started") #message to indicate that the Move_Random has started
        self.generate_map() #generates the map

    #callback function for processing hazard detection messages
    def hazard_callback(self, hazards: HazardDetectionVector):
        if (is_front_hazard_active(hazards)):
            self._start_time = time.perf_counter() #sets timer clock to current time
            self.move(True)
        else:
            self.move(False)

    #function for controlling the robot movement
    def move(self, hazard_detected):
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
    #intialize ros2 client library
    rclpy.init(args=args)

    #create an instance of moveRandom node
    node = MoveRandom()

    #eneter a loop to keep the node running
    rclpy.spin(node)

    rclpy.shutdown() #shutdown ros2 client library

if __name__ == '__main__':
    main()
