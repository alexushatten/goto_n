#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml
import math

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from visualization_msgs.msg import Marker, MarkerArray
from duckietown_utils import load_map



class GoToNNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Set up map
        self.map_name = 'ethz_amod_lab_k31'
        self.map_filename = '/code/catkin_ws/src/goto_n/packages/goto_n/maps/{}.yaml'.format(self.map_name)
        self.map_data = load_map(self.map_filename)

        self.map_matrix, self.matrix_shape, self.tile_size = self.extract_tile_matrix()
        self.node_matrix = self.encode_map_structure(self.map_matrix)
        print (self.node_matrix)

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.callback)

        print("initialized")

    def extract_tile_matrix(self):
        tile_matrix = []
        for tile in self.map_data["tiles"]:
            tile_matrix.append(tile)
        
        tile_matrix = np.array(tile_matrix)
        print(tile_matrix)
        autolab_matrix_shape = tile_matrix.shape

        tile_size = self.map_data["tile_size"]
        print (tile_size)
        return tile_matrix, autolab_matrix_shape, tile_size

    def encode_map_structure(self, tile_matrix):
        row = tile_matrix.shape[0]
        column = tile_matrix.shape[1]
        #print(tile_matrix)

        node_matrix = tile_matrix
        for i in range(0, row):
            for j in range(0, column):
                node_matrix[i,j] = tile_matrix[i,j]
                
                if tile_matrix[i,j] == "asphalt":
                    node_matrix[i,j]= 0
                elif tile_matrix[i,j] == "straight/N" or tile_matrix[i,j] =="straight/S": 
                    node_matrix[i,j]= 1
                elif tile_matrix[i,j] == "straight/W" or tile_matrix[i,j] =="straight/W": 
                    node_matrix[i,j]= 2
                elif tile_matrix[i,j] == "curve_left/N" or tile_matrix[i,j] =="curve_right/E":  
                    node_matrix[i,j]= 3
                elif tile_matrix[i,j] == "curve_left/S" or tile_matrix[i,j] =="curve_right/W": 
                    node_matrix[i,j]= 4
                elif tile_matrix[i,j] == "curve_left/W" or tile_matrix[i,j] =="curve_right/N": 
                    node_matrix[i,j]= 5
                elif tile_matrix[i,j] == "curve_left/E" or tile_matrix[i,j] =="curve_right/S": 
                    node_matrix[i,j]= 6
                elif tile_matrix[i,j] == "3way_left/N" or tile_matrix[i,j] =="3way_right/S": 
                    node_matrix[i,j]= 7
                elif tile_matrix[i,j] == "3way_left/S" or tile_matrix[i,j] =="3way_right/N": 
                    node_matrix[i,j]= 8
                elif tile_matrix[i,j] == "3way_left/W" or tile_matrix[i,j] =="3way_right/E": 
                    node_matrix[i,j]= 9
                elif tile_matrix[i,j] == "3way_left/E" or tile_matrix[i,j] =="3way_right/W": 
                    node_matrix[i,j]= 10
                else:
                    node_matrix[i,j] = 11

        return node_matrix


    def find_current_tile(self,pose_x, pose_y):
        number_of_rows = self.matrix_shape[0]
        #Change representation to crows and colunmns THIS IS MESSY
        tile_column = int((self.tile_size/2) + pose_x/self.tile_size) - 1
        if tile_column < 0:
            tile_column = 0
        tile_row = number_of_rows - int((self.tile_size/2) + pose_y/self.tile_size) - 1
        if tile_row < 0:
            tile_row = 0
        return tile_row, tile_column
    
    def quat_to_euler(self, q):
        sqw = q.w * q.w
        sqx = q.x * q.x
        sqy = q.y * q.y
        sqz = q.z * q.z

        normal = math.sqrt(sqw + sqx + sqy + sqz)
        pole_result = (q.x * q.z) + (q.y * q.w)

        pi = math.pi
        deg = 0

        if (pole_result > (0.5 * normal) or pole_result < (-0.5 * normal)): # singularity at north pole
            #Eulervalue
            rz = 0
            #Convert to deg
            rz = rz + pi/2
            deg = (rz/pi)*180 
            return deg

        r11 = 2*(q.x*q.y + q.w*q.z)
        r12 = sqw + sqx - sqy - sqz
        
        #Eulervalue
        rz = abs(math.atan2( r11, r12 ))

        #Convert to deg
        rz = rz + pi/2
        deg = (rz/pi)*180
        return deg

    def determine_position_tile(self, pose_row, pose_column):
        current_tile_type = self.node_matrix[pose_row, pose_column]
        return int(current_tile_type)

    
    def orientation_correction(self, initial_orientation, current_tile_type):
        orientation = initial_orientation
        if current_tile_type == 0:
            print('You are on Asphalt, call rescue team')
        
        #if it is positioned on the straight N/S tile; default orientation is North
        if current_tile_type == 1:
            orientation_disparity = 0 - orientation
            abs_orientation = abs(orientation_disparity)
            if (abs_orientation < 45 or abs_orientation > 315):
                print('The vehicle is orientated in approximately the correct orientation')
            else:
                print('The current orientation disparity is: {}'.format(orientation_disparity))
                print('Orientating the duckiebot towards north')
                #rotate the duckiebot by orientation disparity to look north

        #if it is positioned on the straigh W/E time; default orientation is East
        elif current_tile_type == 2:
            orientation_disparity = 90 - orientation
            abs_orientation = abs(orientation_disparity)
            if (abs_orientation < 45 or abs_orientation > 315):
                print('The vehicle is orientated in approximately the correct orientation')
            else:
                print('The current orientation disparity is: {}'.format(orientation_disparity))
                print('Orientating the duckiebot towards north')
                #rotate the duckiebot by orientation disparity to look north

        else:
            print("have a nice day")
    
    def callback(self, markerarray):
        marker = markerarray.markers
        for bots in marker: 
            if bots.ns == "duckiebots":
                duckiebot_x = bots.pose.position.x
                duckiebot_y = bots.pose.position.y
                
                duckiebot_orientation= self.quat_to_euler(bots.pose.orientation)
                duckiebot_row, duckiebot_column = self.find_current_tile(duckiebot_x, duckiebot_y)
                
                current_tile_type = self.determine_position_tile(duckiebot_row, duckiebot_column)
                print(current_tile_type)
                self.orientation_correction(duckiebot_orientation,current_tile_type)

if __name__ == '__main__':
    # Initialize the node
    goto_node = GoToNNode(node_name='goto_n')
    # Keep it spinning to keep the node alive
    rospy.spin()