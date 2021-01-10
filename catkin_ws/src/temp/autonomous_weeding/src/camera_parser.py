import cv2
import numpy as np
import rospy 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt

class CameraStreamParser():
    """
    Class for parsing Thorvald's Camera stream
    |Conducts Colour Thresholding
    |Applies Centre crop to image
    """
    def __init__(self):
        """
        Constructor
        |inits ros node camera_parser
        |Published Topic: camera_parser/cropped_img : Image
        |Subscribed Topics: thorvald_001/kinect2_camera/hd/image_color_rect : Image
        """
        #Ros variables
        rospy.init_node('camera_parser')
        self.pub_handle = rospy.Publisher('camera_parser/cropped_img', Image, queue_size=10)
        self.img_sub = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.camera_callback)
        
        #Centre-crop factors
        self.height_factor = 3
        self.width_factor = 1

        #Mean green colour threshold, as defined by empirical search
        self.green_threshold = 43

        #Cv bridge object for converting ROS messages
        self.bridge = CvBridge()
        rospy.spin()

    def publish(self,img):
        """
        Publishes new cropped image
        |img : image to publish
        """
        data = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub_handle.publish(data)

    def centre_crop(self, img, roi_height,roi_width):
        """
        Returns a centre subsection of the image
        img: image to manipulate
        roi_height: height of region of interest
        roi_width: width or region of interest
        """
        assert (img.shape[0]>=roi_height and img.shape[1]>=roi_width), (img.shape[0], roi_height, img.shape[1], roi_width)
        height, width = img.shape[:2]
        left = (width - roi_width)/2
        right = (width + roi_width)/2
        top = (height - roi_height)/2
        bottom  = (height + roi_height)/2
        img_centre = img[top:bottom, left:right]
        return img_centre
    
    
    def mean_colour_check(self, img):
        """
        Returns boolean whether mean green channel > threshold
        |img: img to analyse
        """
        return np.mean(img[:,:,1]) > self.green_threshold
        

    def camera_callback(self, data):
        """
        Image stream callback function
        |data : Image data from topic
        """
        #get dimensions
        height, width = [data.height, data.width]
        #Get raw image as opencv image
        raw_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')        
        #Check if it meets the green threshold
        if self.mean_colour_check(raw_img):
            #Centre crop
            im2pub = self.centre_crop(raw_img, int(height/self.height_factor), int(width/self.width_factor))
            #Publish
            self.publish(im2pub)     

parser = CameraStreamParser()    