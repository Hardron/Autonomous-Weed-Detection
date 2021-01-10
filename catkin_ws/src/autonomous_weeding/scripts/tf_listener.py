#!/usr/bin/env python

import rospy
import tf.transformations as tft
import tf
import numpy as np 
from geometry_msgs.msg import Pose, PoseStamped


class TFListener():
    """
    Transform Listener class that handles transform requests
    """
    def __init__(self):
        """
        Constructor Method
        |Instantiate TF Listener
        |Set rate
        """
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(.5)
    
    def get_transform(self, target_frame, source_frame):
        """
        Method for obtaining the transformation between two coordinate frames
        |target_frame : frame to get relative to the source frame
        |source_frame : frame to get transformation from
        """
        try:
            #target_frame, source_frame
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time())
            return(trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e: 
            print(e)
    
    def transform_pose(self,frame,pose):
        """
        Method for transforming a pose into a new frame
        |frame : frame to transform into
        |pose : pose to transform
        """
        return self.listener.transformPose(frame, pose)
        
