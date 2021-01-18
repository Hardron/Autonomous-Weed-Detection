#!/usr/bin/env python

"""
Movebase Client adapted from
 https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
"""


import rospy
import actionlib
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re
import sys

class MovebaseClient():
    """
    MoveBase client class
    |Loads list of waypoints
    |Sends them to the move_base server
    """
    def __init__(self, goal_file):
        """
        Constructor class
        || goal_file : path to json goal file
        |init rosnode movebase_client_py
        |init movebase client
        |init queue object
        """
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.queue = self.generate_queue(goal_file)

    def generate_queue(self, goal_file):
        """
        Generates ordered list of goals from json file
        |goal_file : path to json file of goals
        |returns ordered list
        """
        queue = []

        with open(goal_file) as f:
            #read in json of goals
            data = json.load(f)
            #This sorts the goals in order of the integer value in their key (i.e. goal2 is the second goal)
            key_list = sorted(list(data.keys()), key=lambda x:int(re.search(r'\d+', x).group()))

        for key in key_list:
            waypoint = data[key]
            goal = MoveBaseGoal() 
            goal.target_pose.header.frame_id = "map"            
            # Set position of goal
            goal.target_pose.pose.position.x = waypoint['position']['x']
            goal.target_pose.pose.position.y = waypoint['position']['y']
            goal.target_pose.pose.position.z = waypoint['position']['z']
            # set orientation of goal
            goal.target_pose.pose.orientation.x = waypoint['orientation']['x']
            goal.target_pose.pose.orientation.y = waypoint['orientation']['y']
            goal.target_pose.pose.orientation.z = waypoint['orientation']['z']
            goal.target_pose.pose.orientation.w = waypoint['orientation']['w']
            queue.append(goal)

        return queue

    def goal_handler(self):
        """
        Main loop
        Passes each goal to server and waits for a resposne
        """
        while not rospy.is_shutdown() and len(self.queue) > 0:            
            # Waits until the action server has started up and started listening for goals.
            self.client.wait_for_server()
            #Get current goal
            goal = self.queue[0]
            #Add timestamp
            goal.target_pose.header.stamp = rospy.Time.now()
            # Sends the goal to the action server.
            self.client.send_goal(goal)
            # Waits for the server to finish performing the action.
            wait = self.client.wait_for_result()
            # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # remove goal from queue
                self.queue.pop(0)
                rospy.loginfo("Goal reached! %d more goals to go..." % len(self.queue))



try:
    rospy.init_node('movebase_client_py')
    try:
        path = sys.argv[1]
        client = MovebaseClient(path)
        rospy.loginfo("Loading waypoints from " + str(path))
        client.goal_handler()
    except IndexError as i:
        rospy.logerr("Error, movebase_client expected a path to goal file!")    
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation test finished.")
