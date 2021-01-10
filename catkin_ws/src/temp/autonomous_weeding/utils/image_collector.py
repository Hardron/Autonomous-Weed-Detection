import rospy
from sensor_msgs.msg import Image
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np


class ImageCollector():
    """
    Class for saving images from the Thorvald camera
    |Node Name: image_saver
    |Subscribed Topics: thorvald_001/kinect2_camera/hd/image_color_rect
    |Dependencies: rospy, sensor_msgs,std_msgs, cv_bridge, uolcmpbase
    """
    def __init__(self):
        """
        Constructor
        Instantiates ROS Node, CvBridge and object variables
        """
        rospy.init_node("image_saver")
        self.image_reader = rospy.Subscriber('thorvald_001/kinect2_camera/hd/image_color_rect', Image, self.image_callback)
        #Create bridge between ROS and OpenCV
        self.bridge = CvBridge()
        self.imgcount = 0
        self.save_path = "./new_dataset/THIRD_ROW_RUN4_thorvald_camera_img_"
        self.file_type = ".jpg"
        self.bridge = CvBridge()
        #Mean green reading to identify plantation
        rospy.spin()
    
    def image_callback(self,data):
        """
        Called when new image data is published
        Displays picture for user via matplot
        data: Image message published by Thorvald
        """
        height, width = [data.height,data.width]
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        save_name = self.save_path + str(self.imgcount) + self.file_type
        self.imgcount +=1
        cv2.imwrite(save_name, img)
              

        

if __name__ == "__main__":
    ic = ImageCollector()





