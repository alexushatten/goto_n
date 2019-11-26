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
from std_msgs.msg import String
from std_msgs.msg import Bool
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from visualization_msgs.msg import Marker, MarkerArray
from duckietown_utils import load_map


class GoToNNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNNode, self).__init__(node_name=node_name)

        # Set up map
        self.map_name = 'ethz_amod_lab_k31'
        self.map_filename = '/code/catkin_ws/src/goto_n/packages/goto_n/maps/{}.yaml'.format(self.map_name)
        self.map_data = load_map(self.map_filename)
        self.map_matrix, self.matrix_shape, self.tile_size = self.extract_tile_matrix()
        self.node_matrix = self.encode_map_structure(self.map_matrix)

        # Create movement_matrixes
        self.go_north= self.go_matrix("north")
        self.go_east= self.go_matrix("east")
        self.go_south= self.go_matrix("south")
        self.go_west= self.go_matrix("west")
        self.go_nowhere = np.eye(self.matrix_shape[0]*self.matrix_shape[1])

        # Create direction possibilities
        self.directions = [self.go_north,self.go_east,self.go_west,self.go_south]
        self.direction_names = ["north","east","west","south"]

        #initialize the published message
        self.msg = None

        #Set up message process
        self.message_recieved = False
        
        # Initialize duckiebots 
        self.autobot_list = rospy.get_param('~duckiebots_list')

        #Initialize termination positions  [Row, Column, Direction]
        self.all_termination_positions = rospy.get_param('~termination_positions_list')
        self.termination = self.extract_termination_tile()
        self.both_termination_types = np.concatenate([self.termination, self.all_termination_positions], axis=1)
        print(self.both_termination_types)
        

        # Start Publisher for each duckiebot
        self.command_publisher=[]
        for autobot in self.autobot_list:
            self.command_publisher.append(rospy.Publisher('/'+autobot+'/movement_commands', Int32MultiArray, queue_size=10))

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
        if len(self.autobot_list) == len(self.all_termination_positions):
            print('\nThere are an equal number of autobots and termination states. Starting Goto_n Node! \n')
        elif len(self.all_termination_positions) > len(self.autobot_list):
            print('\nMissing Autobots. Plese retry localization. \n')
        else: 
            print('\nPlease enter another Termination State \n')    
        
    def request_watchtower_image(self):
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

    def find_current_tile(self, pose_x, pose_y):
        number_of_rows = self.matrix_shape[0]
        number_of_columns = self.matrix_shape[1]
        #Change representation to crows and colunmns THIS IS MESSY
        tile_column = int(round((self.tile_size/2) + pose_x/self.tile_size) - 1)

        if tile_column < 0:
            tile_column = 0
        if tile_column > number_of_columns - 1:
            tile_column = number_of_columns - 1
        tile_row = int((number_of_rows - 1) - round((self.tile_size/2) + pose_y/self.tile_size - 1))
        if tile_row < 0:
            tile_row = 0
        elif tile_row > number_of_rows - 1:
            tile_row = number_of_rows - 1
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

    def cost_matrix(self, termination_point, all_bot_positions, current_bot_position):
        matrix_size = self.matrix_shape[0]*self.matrix_shape[1]
        cost_mat=np.matmul(self.go_north,np.ones((matrix_size,1)))
        cost_mat=np.append(cost_mat,np.matmul(self.go_east,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_west,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_south,np.ones((matrix_size,1))),1)
        cost_mat=np.append(cost_mat,np.matmul(self.go_nowhere,1000*np.ones((matrix_size,1))),1)
        cost_mat[cost_mat < 0.1]=float('inf')

        #Add termination_point
        termination_row = termination_point[0]
        termination_column = termination_point[1]
        termination_direction = termination_point[2]

        #Add 0 cost to desired location
        cell = termination_row*self.matrix_shape[1] + termination_column
        cost_mat[cell,4]=0
        

        #Add inf cost to direction oposite of enddirection in the cell next to it
        if termination_direction == 0:
            neighbourcell = cell - self.matrix_shape[1] 
            cost_mat[neighbourcell,3] = float('inf')
        if termination_direction == 1:
            neighbourcell = cell + 1
            cost_mat[neighbourcell,2] = float('inf')
        if termination_direction == 2:
            neighbourcell = cell - 1
            cost_mat[neighbourcell,1] = float('inf')
        if termination_direction == 3:
            neighbourcell = cell + self.matrix_shape[1] 
            cost_mat[neighbourcell,0] = float('inf')

        #Add inf cost to direction oposite of startdirection in startcell of the currents bot position
        current_name = current_bot_position[0]
        current_init_row = current_bot_position[1]
        current_init_column = current_bot_position[2]
        current_init_direction = current_bot_position[3]
        start_cell = current_init_row*self.matrix_shape[1] + current_init_column

        if current_init_direction == 0:
            cost_mat[start_cell,3] = float('inf')
        if current_init_direction == 1:
            cost_mat[start_cell,2] = float('inf')
        if current_init_direction == 2:
            cost_mat[start_cell,1] = float('inf')
        if current_init_direction == 3:
            cost_mat[start_cell,0] = float('inf')

        for other_bot_position in all_bot_positions:

            other_name = other_bot_position[0]
            #Skip the bot you are currently calculating the map for
            if other_name == current_name:
                continue

            other_init_row = other_bot_position[1]
            other_init_column = other_bot_position[2]
            other_init_direction = other_bot_position[3]

            other_cell = other_init_row*self.matrix_shape[1] + other_init_column
            
            cost_mat[other_cell,other_init_direction] = float('inf')

        return cost_mat

    def value_iteration(self, cost_mat):
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
        iterate = True
        while iterate == True:
            iteration_value+=1
            V_temporary_matrix=V_matrix
            for i in range(0, matrix_size):
                for k in range(0,5):
                    Matrix_for_all_Commands=np.matmul(all_movements_matrix[:,matrix_size*k:matrix_size*(k+1)],V_matrix)
                    V_amound_of_inputs[k]=cost_mat[i,k] + Matrix_for_all_Commands[i]
                V_new_matrix[i]=np.amin(V_amound_of_inputs)
                I_matrix[i]=np.argmin(V_amound_of_inputs)
            V_matrix=V_new_matrix
            if iteration_value==matrix_size:
                iterate = False
        Optimal_movements=I_matrix
        Number_Of_Movements=V_matrix
        Number_Of_Movements[Number_Of_Movements > 1000]=float('inf')

        return Optimal_movements , Number_Of_Movements.astype(int)

    def find_robot_commands(self, bot_position_and_orientation, optimal_movements, number_of_movements):
        name = bot_position_and_orientation[0]
        row = bot_position_and_orientation[1]
        column = bot_position_and_orientation[2]
        orientation = bot_position_and_orientation[3]
        
        cell = row*self.matrix_shape[1] + column
        movement_commands = []
        current_orientation = orientation
        previous_orientation = orientation
        total_number_of_moments = int(number_of_movements[cell])
        for i in range (0, total_number_of_moments):
            compass_movement = optimal_movements[cell]
            if compass_movement == 0:
                transition_point = -self.matrix_shape[1]
            elif compass_movement == 1:
                transition_point = 1
            elif compass_movement == 3:
                transition_point = self.matrix_shape[1]
            elif compass_movement == 2:
                transition_point = -1
            
            if self.node_matrix[row][column] > 6:
                current_tile = self.node_matrix[row][column]
                if previous_orientation ==  optimal_movements[cell]:
                    movement_commands.append(1) #DONT TURN
                elif previous_orientation == 0 or previous_orientation == 3:
                    if abs(optimal_movements[cell]- previous_orientation) == 1:
                        movement_commands.append(2) #RIGHT
                    if abs(optimal_movements[cell] - previous_orientation) == 2:
                        movement_commands.append(0) #LEFT
                
                elif previous_orientation == 1 or previous_orientation == 2:
                    if abs(optimal_movements[cell]- previous_orientation) == 2:
                        movement_commands.append(2) #RIGHT
                    if abs(optimal_movements[cell] - previous_orientation) == 1:
                        movement_commands.append(0) #LEFT

            else:
                movement_commands.append(3) #STRAIGHT

            previous_orientation = optimal_movements[cell]
            cell = cell + transition_point
            row = int(math.floor(cell/self.matrix_shape[1]))
            column = int(cell - row*self.matrix_shape[1])
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
    
    def planner(self, bot_positions):
        termination_positions = self.termination 
        all_termination_positions = self.all_termination_positions    
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
                    continue
                cost_mat = self.cost_matrix(termination_point, changing_bot_positions, current_bot)
                Optimal_movements , Number_Of_Movements = self.value_iteration(cost_mat)
                total_tiles_to_move, movememt_commands = self.find_robot_commands(current_bot, Optimal_movements, Number_Of_Movements)
                different_movement_options.append ([total_tiles_to_move] + [movememt_commands] + [termination_point])
                total_tiles_list.append(total_tiles_to_move)

            minimum_index = total_tiles_list.index(min(total_tiles_list))
            best_choice = different_movement_options[minimum_index]
            changing_bot_positions[i] = [current_bot[0]] + different_movement_options[minimum_index][2]
            message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]]            
            skip_termination.append(termination_positions[minimum_index])
            bot_messages.append(message_to_robot)
            duckiebot_id.append(current_bot[0])
            way_points.append(best_choice[1])
            termination_tiles.append(best_choice[2])
            total_moves_per_bot.append(total_tiles_list[minimum_index])
            total_moves = sum(total_moves_per_bot)
            i += 1
        return bot_messages, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles

    def order_optimization(self, all_bot_positions, duckiebot_id):

        num_of_bots = len(duckiebot_id)
        print('--------------------')
        print(all_bot_positions)
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
        duckie = []
        for bots in marker: 
            if bots.ns == "duckiebots":
                #Set duckiebots position and orientation
                duckiebot_name = bots.id
                duckiebot_x = bots.pose.position.x
                duckiebot_y = bots.pose.position.y

                duckiebot_orientation= self.quat_to_compass(bots.pose.orientation)
                duckie_compass_notation = self.find_compass_notation (duckiebot_orientation)
                
                #Find duckiebots tile column and row
                duckiebot_row, duckiebot_column = self.find_current_tile(duckiebot_x, duckiebot_y)
                
                duckie = [duckiebot_name, duckiebot_row, duckiebot_column, duckie_compass_notation]
                all_bot_positions.append(duckie)
                self.message_recieved = True
                print('Received Message from the Online Localization. Starting Planning Algorithm: \n')
        
        return all_bot_positions, duckie

    def extract_termination_tile(self):
        termination_positions = self.all_termination_positions
        termination_tile_positions = []        

        for termination_position in termination_positions:
            pose_x = termination_position[0]
            pose_y = termination_position[1]
            orientation = termination_position[2]
            termination_row, termination_column = self.find_current_tile(pose_x, pose_y)
            termination_compass_notation = self.find_compass_notation (orientation)
            termination = [termination_row, termination_column, termination_compass_notation]
            termination_tile_positions.append(termination)
        
        print(termination_tile_positions)
        return termination_tile_positions



    def callback(self, markerarray):
        marker = markerarray.markers
        all_bot_positions = []
        if self.message_recieved == False:
            #if does not work anymore, repaste the extract_autobot_data here (minus the return)
            all_bot_positions, duckie = self.extract_autobot_data(marker)

        #print('The starting Positions of all bots are: {}'.format(all_bot_positions))
        bot_ids = self.extract_bots(all_bot_positions)
        #print(all_bot_positions)

        #print(bot_ids)
        
        best_start_config = self.order_optimization(all_bot_positions, bot_ids)
        plan, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles = self.planner(best_start_config)

        print('xxxxxx')
        print(termination_tiles)
        print(self.termination)
        print(self.all_termination_positions)
        print(self.both_termination_types)
        print(plan)
        print('xxxxxxxxxxxxxxx')     
        #this is the code for multiple autobot
        for i in range(0, len(bot_ids)):
            bot_indx=duckiebot_id.index(bot_ids[i])
            duckiebot = duckiebot_id[bot_indx]
            message = way_points[bot_indx]
 
            pum_msg = Int32MultiArray()   
            if self.message_recieved == False:
                print('Empty Message')
            else:
                self.msg = message
                print('The starting Positions of all bots are: {} \n'.format(all_bot_positions))
                print('The waypoint commands sent out to the duckiebot {} are {}. \n'.format(duckiebot, message))
                print('Waypoint Commands sent out to the respective Duckiebots! \n')
            
            rospy.sleep(1)
            pum_msg = Int32MultiArray(data=self.msg)
            self.command_publisher[i].publish(pum_msg)

     
if __name__ == '__main__':
    # Initialize the node
    goto_node = GoToNNode(node_name='goto_n')
    # Keep it spinning to keep the node alive
    rospy.spin()