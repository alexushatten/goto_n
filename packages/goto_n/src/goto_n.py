#!/usr/bin/env python
import cv2
import numpy as np
import os
import rospy
import yaml
import math
from itertools import permutations
import re

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from visualization_msgs.msg import Marker, MarkerArray
from duckietown_utils import load_map

import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)

#NEW FUNCTIONS
from goto_n_transition_matrix import transition_function
from goto_n_value_iteration import go_matrix ,cost_matrix, value_iteration



class GoToNNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNNode, self).__init__(node_name=node_name)

        # Set up map
        self.map_name = 'ethz_amod_lab_k31'
        self.map_filename = '/code/catkin_ws/src/goto_n/packages/goto_n/maps/{}.yaml'.format(self.map_name)
        self.map_data = load_map(self.map_filename)
        self.map_matrix, self.matrix_shape, self.tile_size = self.extract_tile_matrix()
        small_node_matrix = self.encode_map_structure(self.map_matrix)
        self.node_matrix = self.extend_matrix(small_node_matrix)

        # Create movement_matrixes

       
        self.transition_matrix=transition_function(self.matrix_shape)
        self.go_north=go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"north")
        self.go_east= go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"east")
        self.go_south=go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"south")
        self.go_west= go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"west")
        self.go_nowhere = np.eye(2*self.matrix_shape[0]*2*self.matrix_shape[1])
        #Set up message process
        self.message_recieved = False

        #Initialize termination positions  [Row, Column, Direction]
        self.all_termination_positions = rospy.get_param('~termination_positions_list')
        self.termination = self.extract_termination_tile()
        self.all_termination_positions_array = np.array(self.all_termination_positions)
        self.termination_array = np.array(self.termination)
        self.termination_match_array=np.concatenate((self.termination_array,self.all_termination_positions_array),axis=1)
        #### SOMETHING IFFY

        # Start Publisher for each duckiebot
        self.movement_cmd_pub=[]
        self.termination_commands = rospy.Publisher('/goto_n/termination_commands', Float32MultiArray, queue_size=10)

        # Initialize duckiebots 
        self.autobot_list = rospy.get_param('~duckiebots_list')
        for autobot in self.autobot_list:
            self.movement_cmd_pub.append(rospy.Publisher('/autobot{}/movement_commands'.format(autobot), Int32MultiArray, queue_size=10))


        # Start Publisher for termination location


        #Ensure that the planner is correctly set up
        self.proper_initialization()

        print('The Duckiebots given in the list are: {} \n'.format(self.autobot_list))

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.callback)

        # Initialize watchtower request image message
        self.watchtowers_list = rospy.get_param('~watchtowers_list')
        self.image_request=[]
        for watchtower in self.watchtowers_list:
            self.image_request.append(rospy.Publisher('/'+watchtower+'/'+"requestImage",Bool,queue_size=1))

        # Request image from watchtower
        self.request_watchtower_image()

    def proper_initialization(self):
        #checks if the initialization parameters are met
        if len(self.autobot_list) == len(self.all_termination_positions):
            print('\nThere are an equal number of autobots and termination states. Starting Goto_n Node! \n')
        elif len(self.all_termination_positions) > len(self.autobot_list):
            print('\nMissing Autobots. Plese retry localization. \n')
        else: 
            print('\nPlease enter another Termination State \n')    
        
    def request_watchtower_image(self):
        #requests an image from watchtowers in town
        msg_sended = Bool()
        rospy.sleep(1)
        msg_sended.data = True 
        for i in range(0,len(self.watchtowers_list)):
            self.image_request[i].publish(msg_sended)

    def extract_bots(self, all_bot_positions):
        bot_ids = []
        length = len(all_bot_positions)
        for i in range(0,length):
            bot = all_bot_positions[i][0]
            bot_ids.append(bot)
        return bot_ids
 
    def extract_tile_matrix(self):
        tile_matrix = []
        for tile in self.map_data["tiles"]:
            tile_matrix.append(tile)
        tile_matrix = np.array(tile_matrix)
        return tile_matrix, tile_matrix.shape, self.map_data["tile_size"]

    def encode_map_structure(self, tile_matrix):
        row = tile_matrix.shape[0]
        column = tile_matrix.shape[1]

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

    def find_current_tile(self, pose_x, pose_y, orientation):
        number_of_rows = 2*self.matrix_shape[0]
        number_of_columns = 2*self.matrix_shape[1]
        #Get the column
        tile_column = int(round(((self.tile_size/4) + 2*pose_x/self.tile_size) - 1))
        if tile_column < 0:
            tile_column = 0
        if tile_column > number_of_columns - 1:
            tile_column = number_of_columns - 1

        #Get the row
        tile_row = int((number_of_rows - 1) - round(((self.tile_size/4) + 2*pose_y/self.tile_size - 1)))
        if tile_row < 0:
            tile_row = 0
        elif tile_row > number_of_rows - 1:
            tile_row = number_of_rows - 1
        current_tyle_type=self.determine_position_tile( tile_row, tile_column)
        next_column = int(round(((self.tile_size/4) + 2*pose_x/self.tile_size)))
        previous_column = int(round(((self.tile_size/4) + 2*pose_x/self.tile_size-2)))
        next_row = int(number_of_rows - round(((self.tile_size/4) + 2*pose_y/self.tile_size - 1)))
        previous_row = int(number_of_rows -2 - round(((self.tile_size/4) + 2*pose_y/self.tile_size - 1)))
        if previous_column < 0:
            western_tyle_type= - 1
        else:
            western_tyle_type=self.determine_position_tile( tile_row, previous_column)
        if previous_row < 0:
            northern_tyle_type= - 1
        else:
            northern_tyle_type=self.determine_position_tile( previous_row, tile_column)
        if next_row > number_of_rows - 1:
            southern_tyle_type= - 1
        else:
            southern_tyle_type=self.determine_position_tile( next_row, tile_column)
        if next_column > number_of_columns - 1:
            eastern_tyle_type = - 1
        else:
            eastern_tyle_type=self.determine_position_tile( tile_row, next_column)

        if current_tyle_type < 5:
            if orientation == 0 and current_tyle_type != 1:
                if western_tyle_type == 1:
                    tile_column=previous_column
                elif eastern_tyle_type == 1:
                    tile_column=next_column
                elif northern_tyle_type == 1:
                    tile_row=previous_row
                elif southern_tyle_type == 1:
                    tile_row=next_row
            elif orientation == 1 and current_tyle_type != 3:
                if northern_tyle_type == 3:
                    tile_row=previous_row
                elif southern_tyle_type == 3:
                    tile_row=next_row
                elif western_tyle_type == 3:
                    tile_column=previous_column
                elif eastern_tyle_type == 3:
                    tile_column=next_column
            elif orientation == 2 and current_tyle_type != 4:
                if northern_tyle_type == 4:
                    tile_row=previous_row
                elif southern_tyle_type == 4:
                    tile_row=next_row
                elif western_tyle_type == 4:
                    tile_column=previous_column
                elif eastern_tyle_type == 4:
                    tile_column=next_column
            elif orientation == 3 and current_tyle_type != 2:
                if western_tyle_type == 2:
                    tile_column=previous_column
                elif eastern_tyle_type == 2:
                    tile_column=next_column
                elif northern_tyle_type == 2:
                    tile_row=previous_row
                elif southern_tyle_type == 2:
                    tile_row=next_row
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
        return deg #0 is North

    def determine_position_tile(self, pose_row, pose_column):
        current_tile_type = self.node_matrix[pose_row, pose_column]
        return current_tile_type

    def extend_matrix(self, node_matrix):
        extended_matrix=np.zeros((2*self.matrix_shape[0],2*self.matrix_shape[1]))
        for i in range(0,self.matrix_shape[0]):
            for j in range(0, self.matrix_shape[1]):
                if node_matrix[i][j] == 1:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[2,1],[2,1]]
                elif node_matrix[i][j] == 2:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[4, 4],[3, 3]]
                elif node_matrix[i][j] == 3:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[4, 4],[2, 1]]
                elif node_matrix[i][j] == 4:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[2, 1],[3, 3]]
                elif node_matrix[i][j] == 5:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[2, 4],[2, 3]]
                elif node_matrix[i][j] == 6:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[4, 1],[3, 1]]
                elif node_matrix[i][j] == 7:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[5, 0],[6, 7]]
                elif node_matrix[i][j] == 8:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[8, 9],[0, 10]]
                elif node_matrix[i][j] == 9:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[0, 11],[12, 13]]
                elif node_matrix[i][j] == 10:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[14, 15],[16, 0]]
                elif node_matrix[i][j] == 11:
                    extended_matrix[i*2:i*2+2,j*2:j*2+2]=[[17, 18],[19, 20]]
                                   
        return extended_matrix.astype(int)

    def find_robot_commands(self, bot_position_and_orientation, optimal_movements, number_of_movements):
        name = bot_position_and_orientation[0]
        row = bot_position_and_orientation[1]
        column = bot_position_and_orientation[2]
        orientation = bot_position_and_orientation[3]


        cell = row*2*self.matrix_shape[1] + column
        movement_commands = []
        current_orientation = orientation
        previous_orientation = orientation
        total_number_of_moments = int(number_of_movements[cell])
        for i in range (0, total_number_of_moments):
            
            best_option = optimal_movements[cell][0].astype(int)
            tiletype = self.node_matrix[row][column]
            transition_point= self.transition_matrix[best_option][tiletype]
            

            if self.node_matrix[row][column] == 5:
                if optimal_movements[cell] == 2 :
                    movement_commands.append(2) # RIGHT
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(1) #DONT TURN
            elif self.node_matrix[row][column] == 6:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(0) #LEFT
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(2) #RIGHT
            elif self.node_matrix[row][column] == 7:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(1) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(0) #LEFT
            elif self.node_matrix[row][column] == 8:
                if optimal_movements[cell] == 1 :
                    movement_commands.append(0) #DONT TURN
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(1) #LEFT
            elif self.node_matrix[row][column] == 9:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(2) #DONT TURN
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(0) #LEFT
            elif self.node_matrix[row][column] == 10:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(1) #DONT TURN
                elif optimal_movements[cell] == 1 :
                    movement_commands.append(2) #LEFT
            elif self.node_matrix[row][column] == 11:
                if optimal_movements[cell] == 2 :
                    movement_commands.append(1) #DONT TURN
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(0) #LEFT
            elif self.node_matrix[row][column] == 12:
                if optimal_movements[cell] == 1 :
                    movement_commands.append(1) #DONT TURN
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(2) #RIGHT
            elif self.node_matrix[row][column] == 13:
                if optimal_movements[cell] == 1 :
                    movement_commands.append(2) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(0) #RIGHT
            elif self.node_matrix[row][column] == 14:
                if optimal_movements[cell] == 1 :
                    movement_commands.append(0) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(2) #RIGHT
            elif self.node_matrix[row][column] == 15:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(2) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(1) #RIGHT
            elif self.node_matrix[row][column] == 16:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(0) #DONT TURN
                elif optimal_movements[cell] == 1 :
                    movement_commands.append(1) #RIGHT
            elif self.node_matrix[row][column] == 17:
                if optimal_movements[cell] == 1 :
                    movement_commands.append(0) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(2) #RIGHT
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(1) #STRAIGHT
            elif self.node_matrix[row][column] == 18:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(2) #DONT TURN
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(1) #RIGHT
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(0) #STRAIGHT
            elif self.node_matrix[row][column] == 19:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(0) #DONT TURN
                elif optimal_movements[cell] == 1 :
                    movement_commands.append(1) #RIGHT
                elif optimal_movements[cell] == 3 :
                    movement_commands.append(2) #RIGHT
            elif self.node_matrix[row][column] == 20:
                if optimal_movements[cell] == 0 :
                    movement_commands.append(1) #DONT TURN
                elif optimal_movements[cell] == 1 :
                    movement_commands.append(2) #RIGHT
                elif optimal_movements[cell] == 2 :
                    movement_commands.append(0) #RIGHT


            previous_orientation = optimal_movements[cell]
            cell = cell + transition_point
            row = int(math.floor(cell/(2*self.matrix_shape[1])))
            column = int(cell - row*2*self.matrix_shape[1])
        movement_commands.append(4) #STOP
        return total_number_of_moments, movement_commands

    def find_compass_notation(self, deg):
        compass = 0
        #North
        if deg > 315 or deg <= 45:
            compass = 0
        #East
        if deg > 45 and deg <= 135:
            compass = 1
        #South
        if deg > 135 and deg <= 225:
            compass = 3
        #West
        if deg > 225 and deg <= 315:
            compass = 2
        
        return compass
    
    def replace_termination(self, termination_point):
        termination = self.termination                
        termination_point = (np.asarray(termination_point)).astype(float)
        for i in range(0, len(self.termination_match_array)):
            if self.termination_match_array[i][0]==termination_point[0] and self.termination_match_array[i][1]==termination_point[1] and self.termination_match_array[i][2]==termination_point[2]:
                exact_termination_1 = self.termination_match_array[i][3:6]
                exact_termination = exact_termination_1.tolist()
        return exact_termination
    
    def planner(self, bot_positions):
        termination_positions = self.termination 
        skip_termination = []
        changing_bot_positions = bot_positions
        bot_messages = []
        way_points = []
        duckiebot_id = []
        total_moves = 0
        total_moves_per_bot = []
        termination_tiles = []

        i = 0
        for current_bot in bot_positions:
            different_movement_options = []
            total_tiles_list = []
            #Which termination_point it will end op on
            for termination_point in termination_positions:
                if termination_point in skip_termination:
                    total_tiles_list.append(1000)
                    different_movement_options.append ([1000] + [1000] + [termination_point])
                    continue
                else:
                    cost_mat = cost_matrix(self.matrix_shape, self.go_north, self.go_east, self.go_west, self.go_south, self.go_nowhere,termination_point, changing_bot_positions, current_bot)
                    optimal_movements , number_of_movements = value_iteration(self.matrix_shape, self.go_north, self.go_east, self.go_west, self.go_south, self.go_nowhere,cost_mat)
                    total_tiles_to_move, movememt_commands = self.find_robot_commands(current_bot, optimal_movements, number_of_movements)
                    different_movement_options.append ([total_tiles_to_move] + [movememt_commands] + [termination_point])
                    total_tiles_list.append(total_tiles_to_move)

            minimum_index = total_tiles_list.index(min(total_tiles_list))
            best_choice = different_movement_options[minimum_index]
            changing_bot_positions[i] = [current_bot[0]] + different_movement_options[minimum_index][2]
            
            exact_termination = self.replace_termination(best_choice[2])

            
            message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]

            skip_termination.append(termination_positions[minimum_index])
            print(skip_termination)
            bot_messages.append(message_to_robot)
            duckiebot_id.append(current_bot[0])
            way_points.append(best_choice[1])
            termination_tiles.append(exact_termination)
            total_moves_per_bot.append(total_tiles_list[minimum_index])
            total_moves = sum(total_moves_per_bot)
            i += 1
        return bot_messages, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles

    def order_optimization(self, all_bot_positions, duckiebot_id):

        num_of_bots = len(duckiebot_id)
        available_bot_config = all_bot_positions
        all_plans = []
        total_movements = []
        way_point = []
        
        combinations = list(permutations(available_bot_config, num_of_bots))
        
        for combination in combinations:
            array = np.asarray(combination)
            plan, _, way_points, _, total_moves, _  = self.planner(array)
            total_movements.append(total_moves)
            all_plans.append(plan)
            way_point.append(way_points)
        
        min_plan_index = total_movements.index(min(total_movements))
        best_plan = all_plans[min_plan_index]

        best_configuration = np.asarray(combinations[min_plan_index])    
        return best_configuration 

    def extract_autobot_data(self, marker):
        all_bot_positions = []
        bot_ids = []
        for bots in marker: 
            if bots.ns == "duckiebots":
                #Set duckiebots position and orientation
                duckiebot_id = bots.id
                if duckiebot_id in self.autobot_list:
                    duckiebot_x = bots.pose.position.x
                    duckiebot_y = bots.pose.position.y
                    print(duckiebot_x)
                    print(duckiebot_y)
                    duckiebot_orientation= self.quat_to_compass(bots.pose.orientation)
                    duckie_compass_notation = self.find_compass_notation(duckiebot_orientation)
                    
                    #Find duckiebots tile column and row
                    duckiebot_row, duckiebot_column = self.find_current_tile(duckiebot_x, duckiebot_y,duckie_compass_notation)
                    
                    all_bot_positions.append([duckiebot_id, duckiebot_row, duckiebot_column, duckie_compass_notation])
                    bot_ids.append(duckiebot_id)
                    self.message_recieved = True
        
        return all_bot_positions, bot_ids

    def extract_termination_tile(self):
        termination_positions = self.all_termination_positions
        termination_tile_positions = []        

        for termination_position in termination_positions:
            pose_x = termination_position[0]
            pose_y = termination_position[1]
            orientation = termination_position[2]
            termination_compass_notation = self.find_compass_notation (orientation)
            termination_row, termination_column = self.find_current_tile(pose_x, pose_y, termination_compass_notation)
            termination = [termination_row, termination_column, termination_compass_notation]
            termination_tile_positions.append(termination)

        return termination_tile_positions

    def callback(self, markerarray):
        marker = markerarray.markers
        all_bot_positions = []
        termination_publish_list = []
        if self.message_recieved == False:
            #check if it finds the duckiebots and pose
            all_bot_positions, bot_ids = self.extract_autobot_data(marker)
            print('Received Message from the Online Localization. Starting Planning Algorithm: \n')

            best_start_config = self.order_optimization(all_bot_positions, bot_ids)
            plan, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles = self.planner(best_start_config)

            #this is the code for multiple autobot
            for i in range(0, len(bot_ids)):

                bot_indx=duckiebot_id.index(bot_ids[i])
                duckiebot = duckiebot_id[bot_indx]
                duckiebot_float = duckiebot.astype(float)
                message = way_points[bot_indx]
                termination = termination_tiles[bot_indx]
                termination_message = [duckiebot_float] + termination 
                pum_msg = Int32MultiArray()   
                print('The starting Positions of all bots are: {} \n'.format(all_bot_positions))
                print('The waypoint commands sent out to the duckiebot {} are {}. \n'.format(duckiebot, message))
                print('Waypoint Commands sent out to the respective Duckiebots! \n')
                
                rospy.sleep(1)
                pum_msg = Int32MultiArray(data=message)
                self.movement_cmd_pub[i].publish(pum_msg)

                term_msg = Float32MultiArray()
                rospy.sleep(1)
                term_msg = Float32MultiArray(data=termination_message)
                self.termination_commands.publish(term_msg)
                print('Sent out termination message')

     
if __name__ == '__main__':
    # Initialize the node
    goto_node = GoToNNode(node_name='goto_n')
    # Keep it spinning to keep the node alive
    rospy.spin()