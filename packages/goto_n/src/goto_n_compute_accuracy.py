#!/usr/bin/env python
import numpy as np
import os
import rospy
from duckietown import DTROS

from std_msgs.msg import Int32MultiArray, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class GoToNAccuracy(DTROS):
    """GoToNAccuracy

    This node implements the final position control for the Duckiebot as it tries to reach
    its final location. 

    Args:
        GotoNAccuracy(self)

    Subscriber:
        /cslam_markers (:obj:`MarkerArray`): The position/orientation infor from the localisation system
        /goto_n/termination_commands (:obj:'Float32MultiArray'): This subsciber awaits the 
            exact termination location (in global coordinates) from the global planner/server

    Publisher:
        autobot{}/positional_diff (:obj:`Float32MultiArray`): The
            positional difference between the current duckiebot location and the 
            desired termination position (in global coordinates). This is published to
            a topic with the respective autobotID in the name

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNAccuracy, self).__init__(node_name=node_name)

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.location_callback)

        #Subscribe to autobot exact termination_positions
        self.termination_pose_sub = rospy.Subscriber("/goto_n/termination_commands", Float32MultiArray, self.terminationposeCB)

        #Initialize list with the termination positions
        self.termination_positions = []
        
        #Create publisher
        self.command_publisher=[]
        self.autobot_list = rospy.get_param('~duckiebots_list')
        
        #create publisher for all duckiebots localized
        for autobot in self.autobot_list:
            if autobot < 10:
                self.command_publisher.append(rospy.Publisher('/autobot0{}/positional_diff'.format(autobot), Float32MultiArray, queue_size=10))
            else:
                self.command_publisher.append(rospy.Publisher('/autobot{}/positional_diff'.format(autobot), Float32MultiArray, queue_size=10))

        print("Accuracy node initalized")

    def terminationposeCB(self, msg):
        """This function creates a list of all termination positions in global coordinates. 
        
        Args:
            msg (:obj:'Float32MultiArray'): Array with the global termination positions
        """

        #create list with all termination positions
        self.termination_positions.append(msg.data)

    def location_callback(self, msg):
        """This callback function determines the current position whenever it receives an input
        from the localization system. 

        Whenever the node receives a message from the localization_subscriber, it runs the callback 
        function which aims to determine the positional discrepancy between the desired termination
        postion and the current postion of the duckiebot. 

        Args:
            msg (:obj:'Visualization_msgs.msg.MarkerArray'): The marker array that includes the 
                global x & y coordinates of the duckiebot

        Returns:
            Publishes the delta_x and delta_y for the specific duckiebots to the respective topics.

        """
        marker = msg.markers
        
        if self.termination_positions:
            for bots in marker: 
                if bots.ns == "duckiebots": 
                    for i in range(0,len(self.termination_positions)):
                        if bots.id == int(self.termination_positions[i][0]):

                            #extract the relevant information from the marker
                            duckiebot_x = bots.pose.position.x
                            duckiebot_y = bots.pose.position.y
                            termination_x = self.termination_positions[i][1]
                            termination_y = self.termination_positions[i][2]

                            #calculate the positional discrepancy between current and desired
                            delta_x = abs(termination_x - duckiebot_x)
                            delta_y = abs(termination_y - duckiebot_y)
                            termination_message = [delta_x] + [delta_y]

                            #define the messages to be published
                            msg = Float32MultiArray()
                            msg = Float32MultiArray(data=termination_message)
                            self.command_publisher[i].publish(msg)

if __name__ == '__main__':
    # Initialize the node
    goto_accurazy = GoToNAccuracy(node_name='goto_accurazy')
    # Keep it spinning to keep the node alive
    rospy.spin()