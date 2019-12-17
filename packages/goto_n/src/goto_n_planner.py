#!/usr/bin/env python
from itertools import permutations
from goto_n_value_iteration import *
from goto_n_transition_matrix import *
import math

'''
The functions in this file are used to perform the path planning for the localized duckiebots. 
'''

def replace_termination(termination_point, termination, termination_match_array):                
    """
    The purpose of the function is to replace the node_matrix representation of the termination
    state and match it to the corresponding global position (x and y coordinates). The global
    coordinates of the termination state are then used to compute the final accuracy of the duckiebot. 
    
    Args:
        Termination_point  (:obj:'Int32MultiArray): This array contains the specific termination state
            in node notation that is to be replaced by the termination state in global coordinates. 
        Termination (:obj:'Int32MultiArray): An array that contains all the termination states in node notation
        Termination_match_array (:obj:'Float32MultiArray): This array matches the termination in node notation
            to the corresponding ones in global positions. 
    Returns:
       exact_termination (:obj:'Float32MultiArray): A matrix that containes the global x and y
        coordinates of the termination state
    """
    
    termination_point = (np.asarray(termination_point)).astype(float)
    for i in range(0, len(termination_match_array)):
        if termination_match_array[i][0]==termination_point[0] and termination_match_array[i][1]==termination_point[1] and termination_match_array[i][2]==termination_point[2]:
            exact_termination = termination_match_array[i][3:6].tolist()
    
    return exact_termination

def planner(bot_positions, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size):
    """
    This function creates the individual duckiebot waypoint commands that they have to execute when they reach
    the intersections in order to reach the desired termination states. 

    The planner takes all the duckiebot starting positions and all the termination positions and computes the 
    optimal trajectory for each duckiebot. 

    Args:
        bot_positions (:obj:'Int32MultiArray): The array that contains all the duckiebots and their starting
            positions in node notation
        termination_positions (:obj:'Int32MultiArray): This array contains all the termination positions in
            node notation
        termination_match_array (:obj:'Float32MultiArray): This array matches the termination in node notation
            to the corresponding ones in global positions.
        all_movement_matrix (:obj:'Matrix'): A populated movement_matrix that defines all the moves a duckiebot
            can undertake if it in a certain geographical location.
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix
        node_matrix (:obj:'Matrix'): Matrix that has the tile type encoded into it
        transition_matrix (:obj:'Matrix'): Matrix that defines all the possible tiles that can be reached
            for each tile type
        tile_size (:obj:'Float'): The size of the tiles used in the map


    Returns:
        bot_messages (:obj:'Int32MultiArray): A complete array that send out duckiebot id, waypoint commands,
            targeted termination state and total moves for each duckiebot in the planner.
        total_moves_per_bot (:obj:'Int'): The total number of moves that a given duckiebot makes
            A move is defined as the transition from one node to the next
        way_points (:obj:'Int32MultiArray'): An array that consists only of the waypoint commands for each duckiebot 
        duckiebot_id (:obj:'Int32Array'): An array containing the duckiebot ids (same order as waypoints)
        total_moves (:obj:'Int'): The total number of moves that all duckiebots make (collectively)
            A move is defined as the transition from one node to the next
        termination_tiles (:obj:'Float32MultiArray'): This array consists of all the global termination 
            coordinates for the duckiebots (in the same order as duckiebot_id)
        visualization_global_coordinate (:obj:'Float'): These are the waypoint commands sent to RVIZ 
            in order to visualize the planned path. 
    """

    #Initialize the necessary empty arrays
    skip_termination = []
    changing_bot_positions = bot_positions
    bot_messages = []
    way_points = []
    duckiebot_id = []
    total_moves = 0
    total_moves_per_bot = []
    termination_tiles = []
    visualization_global_coordinate = []
    i = 0
    
    #Iterate through the bots
    for current_bot in bot_positions:
        
        #Initialize necessary empty arrays
        different_movement_options = []
        total_tiles_list = []
        list_of_coordinates = []
       
        #Which termination_point it will end up on
        #iterate through the termination points and calculate the total moves for the duckiebots to it
        for termination_point in termination_positions:

            #if a termination point is already used, then it cannot be used by another duckiebot
            if termination_point in skip_termination:
                total_tiles_list.append(1000)
                different_movement_options.append ([1000] + [1000] + [termination_point])
                list_of_coordinates.append(1)
                continue
            else:
                #calculate the planning relevant parameters 
                cost_mat = cost_matrix(matrix_shape, all_movements_matrix, termination_point, changing_bot_positions, current_bot)
                optimal_movements , number_of_movements = value_iteration(matrix_shape, all_movements_matrix, cost_mat)
                total_tiles_to_move, movement_commands, global_coordinates = find_robot_commands(current_bot, optimal_movements, number_of_movements, matrix_shape, node_matrix, tile_size)
                different_movement_options.append ([total_tiles_to_move] + [movement_commands] + [termination_point])
                total_tiles_list.append(total_tiles_to_move)
                list_of_coordinates.append(global_coordinates)

        #deterine index to match duckiebot id to the correct waypoints
        minimum_index = total_tiles_list.index(min(total_tiles_list))
        best_choice = different_movement_options[minimum_index]
        visualization_global_coordinate.append(list_of_coordinates[minimum_index])   
        exact_termination = replace_termination(best_choice[2], termination_positions, termination_match_array)
        
        #generate the message sent to the duckiebot
        message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]
        bot_messages.append(message_to_robot)
        duckiebot_id.append(current_bot[0])
        way_points.append(best_choice[1])
        termination_tiles.append(exact_termination)
        total_moves_per_bot.append(total_tiles_list[minimum_index])
        total_moves = sum(total_moves_per_bot)


    #Implemented to solve corner case of duckiebots/termination states not passing an intersection
    if total_moves >= 2*matrix_shape[0]*2*matrix_shape[1]:
        
        #initialize the necessary arrays
        smallest_bot_index = total_moves_per_bot.index(min(total_moves_per_bot))
        skip_termination = []
        changing_bot_positions = bot_positions
        bot_messages = []
        way_points = []
        duckiebot_id = []
        total_moves = 0
        total_moves_per_bot = []
        termination_tiles = []
        visualization_global_coordinate = []
        i = 0
        
        for current_bot in bot_positions:
            different_movement_options = []
            total_tiles_list = []
            list_of_coordinates = []
            botid= bot_positions[smallest_bot_index]
            for termination_point in termination_positions:
                if termination_point in skip_termination:
                    total_tiles_list.append(1000)
                    different_movement_options.append ([1000] + [1000] + [termination_point])
                    list_of_coordinates.append(1)
                    continue
                else:
                    cost_mat = cost_matrix(matrix_shape, all_movements_matrix, termination_point, changing_bot_positions, current_bot)
                    optimal_movements , number_of_movements = value_iteration(matrix_shape, all_movements_matrix, cost_mat)
                    total_tiles_to_move, movement_commands, global_coordinates = find_robot_commands(current_bot, optimal_movements, number_of_movements, matrix_shape, node_matrix, tile_size)
                    different_movement_options.append ([total_tiles_to_move] + [movement_commands] + [termination_point])
                    total_tiles_list.append(total_tiles_to_move)
                    list_of_coordinates.append(global_coordinates)
            new_list=list(total_tiles_list)
            new_list.sort()

            if current_bot[0]==botid[0]:
                minimum_index = total_tiles_list.index(new_list[1])
            else:
                minimum_index = total_tiles_list.index(min(total_tiles_list))
            best_choice = different_movement_options[minimum_index]
            visualization_global_coordinate.append(list_of_coordinates[minimum_index])   
            exact_termination = replace_termination(best_choice[2], termination_positions, termination_match_array)
            message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]        
            bot_messages.append(message_to_robot)
            duckiebot_id.append(current_bot[0])
            way_points.append(best_choice[1])
            termination_tiles.append(exact_termination)
            total_moves_per_bot.append(total_tiles_list[minimum_index])
            total_moves = sum(total_moves_per_bot)
            i += 1

    return bot_messages, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles, visualization_global_coordinate

def find_robot_commands(bot_position_and_orientation, optimal_movements, number_of_movements, matrix_shape, node_matrix, tile_size):
    """
    This function goes through the optimal path and converts the node to node commands to intersection 
    commands for each robot. It also outputs the relevant global coordinates of the path in order to 
    visualize it in RVIZ. 

    Args:
        bot_positions_and_orientations (:obj:'Int32Array'): The array stores the id, position and orientation
            of all the localized duckiebots
        optimal_movements (:obj:'Int32MultiArray): This multi array contains the optimal waypoint commands for 
            the duckiebots.
        number_of_movements (:obj:'Int'): This integer gives the total number of movements
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix
        node_matrix (:obj:'Matrix'): Matrix that has the tile type encoded into it
        transition_matrix (:obj:'Matrix'): Matrix that defines all the possible tiles that can be reached
            for each tile type
        tile_size (:obj:'Float'): The size of the tiles used in the map


    Returns:
        movement_commands (:obj:'Int32Array'): This array contains the intersection movement commands
            (only the commands to do at the specific intersections)
        total_number_of_moments (:obj:'Int'): This containes the total number of movement commands (only
            intersection commands)
        global_coordinates (:obj:'Float'): These are the global coordinates for each movement commands in order
            to visualize it in RVIZ.
    """
    
    #extract relevant duckiebot information
    name = bot_position_and_orientation[0]
    row = bot_position_and_orientation[1]
    column = bot_position_and_orientation[2]
    orientation = bot_position_and_orientation[3]
    transition_matrix = transition_function(matrix_shape)

    #initialize necessary arrays
    cell = row*2*matrix_shape[1] + column
    movement_commands = []
    global_coordinates = []
    x_coordinate, y_coordinate = node_to_globalcoordinates(row, column, tile_size, matrix_shape)
    global_coordinates.append([x_coordinate, y_coordinate])

    total_number_of_moments = int(number_of_movements[cell])
    
    
    #Iterate through the total number of movements, replacing
    for i in range (0, total_number_of_moments):
        
        best_option = optimal_movements[cell][0].astype(int)
        tiletype = node_matrix[row][column]
        transition_point= transition_matrix[best_option][tiletype]
        
        #replace the node commands with the cooresponding intersection command depending on 
        #the current tile position 
        if optimal_movements[cell] == 0 :
            if node_matrix[row][column] in (6,16,19):
                movement_commands.append(0) #LEFT
            elif node_matrix[row][column] in (7,10,20):
                movement_commands.append(1) #STRAIGHT
            elif node_matrix[row][column] in (9,15,18):
                movement_commands.append(2) #RIGHT
        elif optimal_movements[cell] == 1 :
            if node_matrix[row][column] in (8,14,17):
                movement_commands.append(0) #LEFT
            elif node_matrix[row][column] in (12,16,19):
                movement_commands.append(1) #STRAIGHT
            elif node_matrix[row][column] in (10,13,20):
                movement_commands.append(2) #RIGHT
        elif optimal_movements[cell] == 2 :
            if node_matrix[row][column] in (7,13,20):
                movement_commands.append(0) #LEFT
            elif node_matrix[row][column] in (11,15,18):
                movement_commands.append(1) #STRAIGHT
            elif node_matrix[row][column] in (5,14,17):
                movement_commands.append(2) #RIGHT
        elif optimal_movements[cell] == 3 :
            if node_matrix[row][column] in (9,11,18):
                movement_commands.append(0) #LEFT
            elif node_matrix[row][column] in (5,8,17):
                movement_commands.append(1) #STRAIGHT
            elif node_matrix[row][column] in (6,12,19):
                movement_commands.append(2) #RIGHT

        cell = cell + transition_point
        row = int(math.floor(cell/(2*matrix_shape[1])))
        column = int(cell - row*2*matrix_shape[1])
        x_coordinate, y_coordinate = node_to_globalcoordinates(row, column, tile_size, matrix_shape)
        global_coordinates.append([x_coordinate, y_coordinate])
    movement_commands.append(4) #STOP
    
    return total_number_of_moments, movement_commands, global_coordinates

def node_to_globalcoordinates (row, column, tile_size, matrix_shape):
    """
    This function takes the node notation of a position and returns the corresponding global
    coordinates that are then sent on to RVIZ.

    Args:
        row (:obj:'Int'): Row of node notation
        column (:obj:'Int'): Column of node notation
        tile_size (:obj:'Float'): The size of the tiles used in the map
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix

    Returns:
        x (:obj:'Float): global x - coordinate
        y (:obj:'Float): global y - coordinate

    """
    y = matrix_shape[0]*tile_size-(tile_size/2)*row-(tile_size/4)
    x = (tile_size/2)*column+(tile_size/4)
    
    return x, y

def order_optimization(all_bot_positions, duckiebot_id, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size):
    """
    This function aims to determine the optimal combination of termination and duckiebots. It 
    creates a list of permutations that iterate through all combinations of duckiebots -> termination
    states and outputs the combination that requires the least moves. This is then used in the planner.  

    Args:
        all_bot_positions (:obj:'Int32MultiArray): The array that contains all the duckiebots and their starting
            positions in node notation
        duckiebot_id (:obj:'Int32Array'): This array contains the duckiebot IDs
        termination_positions (:obj:'Int32MultiArray): This array contains all the termination positions in
            node notation
        termination_match_array (:obj:'Float32MultiArray): This array matches the termination in node notation
            to the corresponding ones in global positions.
        all_movement_matrix (:obj:'Matrix'): A populated movement_matrix that defines all the moves a duckiebot
            can undertake if it in a certain geographical location.
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix
        node_matrix (:obj:'Matrix'): Matrix that has the tile type encoded into it
        transition_matrix (:obj:'Matrix'): Matrix that defines all the possible tiles that can be reached
            for each tile type
        tile_size (:obj:'Float'): The size of the tiles used in the map

    Returns:
        best_configuration (:obj:'Int32MultiArray'): This array contains the sequence of termination states
            that, matched up with the standard order of duckiebots in the planner, outputs the optimal
            movement commands 
    """
    #Initialize the necessary arrays
    num_of_bots = len(duckiebot_id)
    available_bot_config = all_bot_positions
    all_plans = []
    total_movements = []
    way_point = []
    all_termination_tiles =[]
    
    #create all the possible termination state/duckiebot combinations
    combinations = list(permutations(available_bot_config, num_of_bots))
    
    #iterate through these combinations to find the optimal commands
    for combination in combinations:
        array = np.asarray(combination)
        plan, _, way_points, _, total_moves, _, _  = planner(array, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size)
        total_movements.append(total_moves)
        all_plans.append(plan)
        way_point.append(way_points)
    
    min_plan_index = total_movements.index(min(total_movements))
    best_plan = all_plans[min_plan_index]

    best_configuration = np.asarray(combinations[min_plan_index])    
    return best_configuration 
