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

#To print whole array
import sys
np.set_printoptions(threshold=sys.maxsize)


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
        print (self.matrix_shape)
        self.node_matrix = self.encode_map_structure(self.map_matrix)
        print (self.node_matrix)
        
        # Create movement_matrixes
        self.go_north= self.go_matrix("north")
        self.go_east= self.go_matrix("east")
        self.go_south= self.go_matrix("south")
        self.go_west= self.go_matrix("west")

        # Create direction possibilities
        self.directions = [self.go_north,self.go_east,self.go_west,self.go_south]
        self.direction_names = ["north","east","west","south"]
        self.direction_names_reversed = ["south","west","east","north"]
        ###################3TEST
        oreintation = "south"
        row = 3
        column = 2
        dirs = self.find_possible_directions(oreintation,row, column)
        print(dirs)
        ####################

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
        return tile_matrix, autolab_matrix_shape, tile_size

    def encode_map_structure(self, tile_matrix):
        row = tile_matrix.shape[0]
        column = tile_matrix.shape[1]
        #print(tile_matrix)

        node_matrix = np.zeros((self.matrix_shape[0],self.matrix_shape[1]))
        for i in range(0, row):
            for j in range(0, column):
                
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
    
    def quat_to_compass(self, q):
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
        rz = math.atan2( r11, r12 )
        
        #Convert to compasscoordinates 90 being east
        if rz > 0:
            rz = rz - 2*pi

        rz = abs(rz)
        rz = rz + pi/2
        deg = (rz/pi)*180
        deg = deg % 360
        return deg

    def determine_position_tile(self, pose_row, pose_column):
        current_tile_type = self.node_matrix[pose_row, pose_column]
        return current_tile_type

    
    def orientation_correction(self, orientation, current_tile_type):

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
    
    def go_matrix(self,direction):
        matrix_size = self.matrix_shape[0]*self.matrix_shape[1]
        movement_matrix = np.zeros((matrix_size,matrix_size))
        nodes = np.array(self.node_matrix).flatten()
        transition_point = 0

        #Direction of movement init
        if direction == "north":
            valid_nodes = [1,4,6,7,8,10,11]
            transition_point = -self.matrix_shape[1]
        elif direction == "east":
            valid_nodes = [2,4,5,8,9,10,11]
            transition_point = 1
        elif direction == "south":
            valid_nodes = [1,3,5,7,8,9,11]
            transition_point = self.matrix_shape[1]
        elif direction == "west":
            valid_nodes = [2,3,6,7,9,10,11]
            transition_point = -1
        else:
            print("unknown direction choise")

        #Create movemet matrix
        for i in range(0,matrix_size):
            for y in valid_nodes: 
                if nodes[i] == y:
                    movement_matrix[i,i + transition_point] = 1
        return movement_matrix

    def find_possible_directions(self,in_orientation,row, column):
        cell = row*self.matrix_shape[1] + column
        matrix_size = self.matrix_shape[0]*self.matrix_shape[1]
        possible_movements = []
        possible_direction = self.directions
        name_of_possible_direction = self.direction_names
        total_possible_directions = 4
        i = 0

        for dir in self.direction_names_reversed:
            if dir == in_orientation:
                del possible_direction[i]
                del name_of_possible_direction[i]
                total_possible_directions -=1
            i += 1

        for i in range(total_possible_directions):
            for j in range (0,matrix_size):
                if possible_direction[i][cell][j] > 0:
                    #FIND A WAY TO CONVERT THIS>
                    
                    #new_row = j - 
                    #new_column = j % self.matrix_shape[0]
                    #one_possible_dir = [name_of_possible_direction[i],new_row,new_column]

        return one_possible_dir
    
    def callback(self, markerarray):
        marker = markerarray.markers
        for bots in marker: 
            if bots.ns == "duckiebots":
                #Set duckiebots position and orientation
                duckiebot_x = bots.pose.position.x
                duckiebot_y = bots.pose.position.y
                duckiebot_orientation= self.quat_to_compass(bots.pose.orientation)
                
                #Find duckiebots tile column and row
                duckiebot_row, duckiebot_column = self.find_current_tile(duckiebot_x, duckiebot_y)
                #Find what type of tile robot is on
                current_tile_type = self.determine_position_tile(duckiebot_row, duckiebot_column)
                #Correct orientation, if robot is not facing any of the directions possible
                ###################################### TODOLATER! self.orientation_correction(duckiebot_orientation,current_tile_type)

            


if __name__ == '__main__':
    # Initialize the node
    goto_node = GoToNNode(node_name='goto_n')
    # Keep it spinning to keep the node alive
    rospy.spin()