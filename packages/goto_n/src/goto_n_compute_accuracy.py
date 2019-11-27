#!/usr/bin/env python
import numpy as np
import os
import rospy
from duckietown import DTROS

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class GoToNAccuracy(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNAccuracy, self).__init__(node_name=node_name)

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.location_callback)

        #Subscribe to autobot exact termination_positions
        self.termination_pose_sub = rospy.Subscriber("/goto_n/termination_commands", Float32MultiArray, self.terminationposeCB)

        self.termination_positions = []
        #Create publisher
        self.command_publisher=[]
        self.autobot_list = rospy.get_param('~duckiebots_list')
        for autobot in self.autobot_list:
            self.command_publisher.append(rospy.Publisher('/autobot{}/positional_diff'.format(autobot), Float32MultiArray, queue_size=10))

        print("Accuracy node initalized")

    def terminationposeCB(self, msg):
        self.termination_positions.append(msg.data)

    def location_callback(self, msg):
        marker = msg.markers
        if self.termination_positions:
            for bots in marker: 
                if bots.ns == "duckiebots": 
                    for i in range(0,len(self.termination_positions)):
                        if bots.id == int(self.termination_positions[i][0]):

                            duckiebot_x = bots.pose.position.x
                            duckiebot_y = bots.pose.position.y
                            termination_x = self.termination_positions[i][1]
                            termination_y = self.termination_positions[i][2]

                            delta_x = abs(termination_x - duckiebot_x)
                            delta_y = abs(termination_y - duckiebot_y)
                            termination_message = [delta_x] + [delta_y]

                            msg = Float32MultiArray()
                            msg = Float32MultiArray(data=termination_message)
                            self.command_publisher[i].publish(msg)
                            print('FOR autobot{}: The delta x is: {}: the delta y is: {}\n'.format(bots.id,delta_x, delta_y))



               

if __name__ == '__main__':
    # Initialize the node
    goto_accurazy = GoToNAccuracy(node_name='goto_accurazy')
    # Keep it spinning to keep the node alive
    rospy.spin()