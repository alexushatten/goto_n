#!/usr/bin/env python
import numpy as np
import os
import rospy
from duckietown import DTROS

from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class GoToNAccuracy(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNAccuracy, self).__init__(node_name=node_name)

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.location_callback)

        #Create publisher
        self.command_publisher=[]
        self.autobot_list = rospy.get_param('~duckiebots_list')
        for autobot in self.autobot_list:
            self.command_publisher.append(rospy.Publisher('/'+autobot+'/positional_diff', Int32MultiArray, queue_size=10))

        print("Accuracy node initalized")

    def location_callback(self, msg):
        marker = msg.markers
        for bots in marker: 
            if bots.ns == "duckiebots":
                #Set duckiebots position and orientation
                duckiebot_name = bots.id
                duckiebot_x = bots.pose.position.x
                duckiebot_y = bots.pose.position.y


               

if __name__ == '__main__':
    # Initialize the node
    goto_accurazy = GoToNAccuracy(node_name='goto_accurazy')
    # Keep it spinning to keep the node alive
    rospy.spin()