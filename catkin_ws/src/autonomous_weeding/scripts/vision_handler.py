#!/usr/bin/env python

import numpy as np 
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes
import copy
import tf.transformations as tft
import tf
from tf_listener import TFListener
from math import atan2

class VisionHandler():
    """
    Class that handles Spraying based on BoundingBoxes
    |Node name: vision_handler
    |Published Topics: vision_handler/is_spraying : Bool
    |Subscribed Topics: darknet_ros/bounding_boxes | /thorvald_001/odometry/base_raw
    |Dependencies: rospy, geometry_msgs, std_msgs, darknet_ros_msgs, nav_msgs
    """
    def __init__(self):
        """
        Constructor Method
        Instantiates ROS node, defines publisher and subscribers
        Instantiates location variables
        """
        #List of objects to spray
        self.weed_classes = {'easy weed', 'medium weed', 'hard weed'}
        #ROS node params
        rospy.init_node('vision_handler')
        self.pub_handle = rospy.Publisher('vision_handler/is_spraying', Bool, queue_size=10)
        self.bounding_boxes = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.vision_callback, queue_size=10)
        self.odom_listener = rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.odom_callback)

        self.tflistener = TFListener()
        self.spray_frame = 'thorvald_001/sprayer'
        self.base_frame = 'thorvald_001/base_link'
        self.camera_frame = "thorvald_001/kinect2_rgb_optical_frame" 
        self.map_frame = "map"

        #Link Positions on Model
        self.CAMERA_RELIATIVE_POSTION= 0.45
        self.SPRAYER_RELATIVE_POSITION = -0.45
        self.SPRAY_LOCATION_LEEWAY = 0.02
        self.SPRAY_LOCATION_LEEWAY_Y = 0.05
        
        #Current base location
        self.curr_base_pose = PoseStamped()

        #Current location of links
        self.current_spray_pose = PoseStamped()
        self.current_camera_pose = PoseStamped()

        self.current_spray_pose.pose.orientation.w = 1.0
        self.current_camera_pose.pose.orientation.w=1.0

        #vision callback rate limiter variables
        self.callback_count = 0
        self.callback_interval = 5
        self.max_queue_length = 150
        self.queue = []

        self.spray = rospy.ServiceProxy("thorvald_001/spray", Empty)
        rospy.spin()

    def publish(self, is_spraying):
        """
        Publishes Boolean is_spraying
        """
        self.pub_handle.publish(is_spraying)

    def call_spray(self):
        """
        Calls the spray service
        """
        self.publish(True)
        self.spray()
        #Start the vehicle again
        self.publish(False)

    def check_for_weed(self, boxes):
        """
        Checks for the presence of weeds
        |boxes : List of bounding boxes
        """
        if len(self.queue) > self.max_queue_length:
            self.queue.pop(0)
        for each in boxes:            
            #Check if object is a weed
            if each.Class in self.weed_classes:
                #Deep copy of current camera pose
                self.queue.append(copy.deepcopy(self.current_camera_pose))
                #TODO IF YOU WANT TO DO MICRO MOVEMENT, YOU CAN'T BREAK HERE!
                break

    def check_for_spray(self):
        """
        Checks if sprayer is now in the same location as a weed
        """                
        #get spray position
        sprayx = self.current_spray_pose.pose.position.x
        sprayy = self.current_spray_pose.pose.position.y
        #For each weed we've seen, 
        for idx,weed_loc in enumerate(self.queue):
            #check if the spray is at roughly the same location as a weed
            if (abs(sprayx-weed_loc.pose.position.x)<self.SPRAY_LOCATION_LEEWAY) and (abs(sprayy-weed_loc.pose.position.y)<self.SPRAY_LOCATION_LEEWAY_Y):
                #Then spray and remove weed from list
                self.queue.pop(idx)
                self.call_spray()


    def vision_callback(self, data):
        """
        Callback function for bounding boxes subscriber
        |data : list of BoundingBoxes objects
        """
        self.callback_count += 1
        #Rate limit
        if self.callback_count % self.callback_interval == 0:
            boxes = data.bounding_boxes
            self.check_for_weed(boxes)
        

    def odom_callback(self, data):
        """
        Callback function for odometry subscriber
        |data : Odometry data (Pose, Twist)
        """
        #Empty spray pose
        spray_wrt_map = PoseStamped()
        spray_wrt_map.header.frame_id = self.spray_frame
        spray_wrt_map.pose.orientation.w = 1
        #Get spray pose in map frame
        spray_in_map = self.tflistener.transform_pose(self.map_frame, spray_wrt_map)
        #Empty camera pose
        cam_wrt_map = PoseStamped()
        cam_wrt_map.header.frame_id = self.camera_frame
        cam_wrt_map.pose.orientation.w = 1
        #Get camera in map frame
        cam_in_map = self.tflistener.transform_pose(self.map_frame, cam_wrt_map)
        
        self.current_camera_pose = cam_in_map
        self.current_spray_pose = spray_in_map
        
        #Check for collision
        self.check_for_spray()

vh = VisionHandler()
