#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml
import math

from std_msgs.msg import Int16
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
        self.go_nowhere = np.eye(self.matrix_shape[0]*self.matrix_shape[1])
        # Create direction possibilities
        self.directions = [self.go_north,self.go_east,self.go_west,self.go_south]
        self.direction_names = ["north","east","west","south"]
        self.direction_names_reversed = ["south","west","east","north"]

        ##TEST
        init_row = 1
        init_column = 2
        init_deg = 3
        termination_row = 4
        termination_column = 5
        ####################

        self.cost_mat = self.cost_matrix(termination_row, termination_column)
        self.Optimal_movements , self.Number_Of_Movements = self.value_iteration()

        self.movememt_commands = self.find_robot_commands(init_deg,init_row,init_column)
        print (self.movememt_commands)
        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.callback)
        
        #Publisher to publish orientation correction commands
        self.orientation_publisher = rospy.Publisher('/autobot/orientation_correction', Int16, queue_size=10)
        
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

    def orientation_correction(self, orientation, initial_tile_type):
        
        #defining all orientation boundaries in degrees
        north = 360
        north_lb = 30
        north_ub = 330
        south = 180
        south_ub = 210
        south_lb = 150
        east = 90
        east_ub = 120
        east_lb = 60
        west = 270
        west_ub = 300
        west_lb = 240

        required_rotation = 0

        #iterating through the different tile position and reorientating duckiebot
        #orientation order of priority: N -> E -> S -> W
        #The rotation direction is defined as clock-wise
        if initial_tile_type == 0:
            print('You are on Asphalt, call rescue team')
        
        #if it is positioned on the straight N/S tile; default orientation is North

        if initial_tile_type == 1:
            #Possible Orientations: N, S
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately north. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately south. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < south_lb and orientation > north_lb):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    #print('Orientating the Duckiebot by {} too face {}.'format(required_rotation, relevant_direction))
                
                elif (orientation < north_ub and orientation > south_lb):
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
            

        if initial_tile_type == 2: 
            #Possible Orientations: E, W
                       
            if (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < west_lb and orientation > east_lb):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation > west_ub or orientation < east_lb):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))

                    if (orientation > west_ub):
                        required_rotation = (360 - orientation) + 90
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                    elif (orientation < east_lb):
                        required_rotation = 90 - orientation
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
            

        if initial_tile_type == 3:
            #Possible Orientations: S, W 
                        
            if (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < west_lb and orientation > south_lb):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation > west_ub or orientation < south_lb):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))

                    if (orientation > west_ub):
                        required_rotation = (360 - orientation) + 180
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                    elif (orientation < south_lb):
                        required_rotation = 180 - orientation
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
            
        
        if initial_tile_type == 4:
            #Possible Orientations: N, E

            if (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < north_ub and orientation > east_lb):
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < east_lb and orientation > north_lb):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 90 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        


        if initial_tile_type == 5:
            #Possible Orientations: E, S

            if (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < south_lb and orientation > east_ub):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < east_lb and orientation > south_ub):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))

                    if (orientation > south_un):
                        required_rotation = (360 - orientation) + 90
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                    elif (orientation < east_lb):
                        required_rotation = 90 - orientation
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
            

        if initial_tile_type == 6:
            #Possible Orientation; N, W
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < north_ub and orientation > west_ub):
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < west_lb and orientation > north_lb):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                else:
                    print('Restart Localization')
                    exit_node()        
            

        if initial_tile_type == 7:
            #Possible orientation Directions: N, S, W
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < south_lb and orientation > north_lb):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < west_lb or orientation > south_ub):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                elif (orientation < north_ub and orientation > west_ub) :
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
        
        if initial_tile_type == 8:
            #Possible orientation Directions: N, E, S
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            elif (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < east_lb and orientation > north_lb):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 90 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < south_lb or orientation > east_ub):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                elif (orientation < north_ub and orientation > south_ub) :
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        

        if initial_tile_type == 9:
            #Possible orientation Directions: E, S, W
            
            if (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < south_lb and orientation > east_lb):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < west_lb or orientation > south_ub):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                elif (orientation > west_ub or orientation < east_lb) :
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))
                    
                    if (orientation > west_ub):
                        required_rotation = (360 - orientation) + 90
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                    
                    elif (orientation < east_lb):
                        required_rotation = 90 - orientation
                        print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                else:
                    print('Restart Localization')
                    exit_node()        
            
        
        if initial_tile_type == 10:
            #Possible orientation Directions: N, E, W
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            elif (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < east_lb and orientation > north_lb):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 90 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < west_lb or orientation > east_ub):
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                elif (orientation < north_ub and orientation > west_ub) :
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    exit_node()        
            

        if initial_tile_type == 11:
            #Possible orientation Directions: N, E, S, W
            
            if (orientation < north_lb or orientation > north_ub):
                print('The duckiebot is orientated approximately North. No further changes made.')
            elif (orientation < east_ub and orientation > east_lb):
                print('The duckiebot is orientated approximately East. No further changes made.')
            elif (orientation < south_ub and orientation > south_lb):
                print('The duckiebot is orientated approximately South. No further changes made.')
            elif (orientation < west_ub and orientation > west_lb):
                print('The duckiebot is orientated approximately West. No further changes made.')
            else:
                print('The current orientation is: {}'.format(orientation))
                
                #check closest admissible direction:
                
                if (orientation < east_lb and orientation > north_lb):
                    relevant_direction = 'East'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 90 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < south_lb or orientation > east_ub):
                    relevant_direction = 'South'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 180 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))
                
                elif (orientation < west_lb and orientation > south_ub) :
                    relevant_direction = 'West'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 270 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                elif (orientation < north_ub and orientation > west_ub) :
                    relevant_direction = 'North'
                    print('Orientating to {}'.format(relevant_direction))
                    required_rotation = 360 - orientation
                    print('Orientating the Duckiebot by {} too face {}.'.format(required_rotation, relevant_direction))

                else:
                    print('Restart Localization')
                    self.exit_node()        
            

        else:
            print("Please Restart Program")

    
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

    def cost_matrix(self, termination_row, termination_column):
        matrix_size = self.matrix_shape[0]*self.matrix_shape[1]
        cost_mat=np.matmul(self.go_north,np.ones((matrix_size,1)))
        cost_mat=np.append(cost_mat,np.matmul(self.go_east,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_west,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_south,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_nowhere,1000*np.ones((matrix_size,1))),1)
        cost_mat[cost_mat < 0.1]=float('inf')
        cell = termination_row*self.matrix_shape[1] + termination_column
        cost_mat[cell,4]=0
        return cost_mat

    def value_iteration(self):
        matrix_size = self.matrix_shape[0]*self.matrix_shape[1]
        V_matrix = np.zeros((matrix_size,1))
        V_new_matrix = np.zeros((matrix_size,1))
        I_matrix =np.zeros((matrix_size,1))
        V_temporary_matrix=100*np.ones((matrix_size,1))
        V_amound_of_inputs=np.zeros((5,1))
        iteration_value=0
        all_movements_matrix = np.append(self.go_north, self.go_east, 1)
        all_movements_matrix = np.append(all_movements_matrix, self.go_west, 1)
        all_movements_matrix = np.append(all_movements_matrix, self.go_south, 1)
        all_movements_matrix = np.append(all_movements_matrix, self.go_nowhere, 1)
        while 1==1:
            iteration_value+=1
            V_temporary_matrix=V_matrix
            for i in range(0, matrix_size):
                for k in range(0,5):
                    Matrix_for_all_Commands=np.matmul(all_movements_matrix[:,matrix_size*k:matrix_size*(k+1)],V_matrix)
                    V_amound_of_inputs[k]=self.cost_mat[i,k] + Matrix_for_all_Commands[i]
                V_new_matrix[i]=np.amin(V_amound_of_inputs)
                I_matrix[i]=np.argmin(V_amound_of_inputs)
            V_matrix=V_new_matrix
            print(iteration_value)
            if iteration_value==200:
                print("reached maximum iterations")
                break
        Optimal_movements=I_matrix
        Number_Of_Movements=V_matrix
        Number_Of_Movements[Number_Of_Movements > 1000]=float('inf')

        print(Optimal_movements)
        print(Number_Of_Movements)
        return Optimal_movements , Number_Of_Movements.astype(int)

    def find_robot_commands(self, orientation, row, column):
        cell = row*self.matrix_shape[1] + column
        movement_commands = []
        current_orientation = orientation
        previous_orientation = orientation
        for i in range (0, self.Number_Of_Movements[cell]):
            print(i)
            compass_movement = self.Optimal_movements[cell]
            if compass_movement == 0:
                transition_point = -self.matrix_shape[1]
            elif compass_movement == 1:
                transition_point = 1
            elif compass_movement == 3:
                transition_point = self.matrix_shape[1]
            elif compass_movement == 2:
                transition_point = -1
            
            if self.node_matrix[row][column] > 6:
                print(i)
                current_tile = self.node_matrix[row][column]
                if previous_orientation ==  self.Optimal_movements[cell]:
                    movement_commands.append("dont_turn")
                elif previous_orientation == 0 or previous_orientation == 3:
                    if abs(self.Optimal_movements[cell]- previous_orientation) == 1:
                        movement_commands.append("right")
                    if abs(self.Optimal_movements[cell] - previous_orientation) == 2:
                        movement_commands.append("left")
                
                elif previous_orientation == 1 or previous_orientation == 2:
                    print(self.Optimal_movements[cell] - previous_orientation)
                    if abs(self.Optimal_movements[cell]- previous_orientation) == 2:
                        movement_commands.append("right")
                    if abs(self.Optimal_movements[cell] - previous_orientation) == 1:
                        movement_commands.append("left")

            else:
                movement_commands.append("straight")

            previous_orientation = self.Optimal_movements[cell]
            cell = cell + transition_point
            row = int(math.floor(cell/self.matrix_shape[1]))
            column = int(cell - row*self.matrix_shape[1])
        movement_commands.append("stop")
        return movement_commands

            
            


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
                    print(i)
                    new_row = int(math.floor(j/self.matrix_shape[1]))
                    new_column = int(j - new_row*self.matrix_shape[1])
                    one_possible_move = [name_of_possible_direction[i],new_row,new_column]
                    possible_movements.append(one_possible_move)

        return possible_movements
    
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