#!/usr/bin/env python
import numpy as np
import os
import rospy
from duckietown import DTROS

from visualization_msgs.msg import Marker, MarkerArray


class GoToNAccuracy(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNAccuracy, self).__init__(node_name=node_name)

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.location_callback)


        print("Accuracy node initalized")

    def location_callback(self, msg):
        print("got message")

if __name__ == '__main__':
    # Initialize the node
    goto_accurazy = GoToNAccuracy(node_name='goto_accurazy')
    # Keep it spinning to keep the node alive
    rospy.spin()