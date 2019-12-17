#!/usr/bin/env python
from itertools import permutations
from goto_n_value_iteration import *
from goto_n_transition_matrix import *
import math


def replace_termination(termination_point, termination, termination_match_array):                
    termination_point = (np.asarray(termination_point)).astype(float)
    for i in range(0, len(termination_match_array)):
        if termination_match_array[i][0]==termination_point[0] and termination_match_array[i][1]==termination_point[1] and termination_match_array[i][2]==termination_point[2]:
            exact_termination = termination_match_array[i][3:6].tolist()
    return exact_termination

def planner(bot_positions, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size, optimal_moves):
    skip_termination = []
    changing_bot_positions = bot_positions
    bot_messages = []
    way_points = []
    duckiebot_id = []
    total_moves = 0
    total_moves_per_bot = []
    termination_tiles = []
    picked_global_coordinate = []
    bots = []
    terminations = []
    i = 0
    for current_bot in bot_positions:
        different_movement_options = []
        total_tiles_list = []
        list_of_coordinates = []
        #Which termination_point it will end op on
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
        new_list=total_tiles_list
        new_list.sort()
        minimum_index = total_tiles_list.index(min(total_tiles_list))
        best_choice = different_movement_options[minimum_index]
        picked_global_coordinate.append(list_of_coordinates[minimum_index])   
        exact_termination = replace_termination(best_choice[2], termination_positions, termination_match_array)
        message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]
        bot_messages.append(message_to_robot)
        duckiebot_id.append(current_bot[0])
        way_points.append(best_choice[1])
        termination_tiles.append(exact_termination)
        total_moves_per_bot.append(total_tiles_list[minimum_index])
        total_moves = sum(total_moves_per_bot)
        bots.append(current_bot[0])
        terminations.append(best_choice[2])
        i += 1
    """   if total_moves >= 2*matrix_shape[0]*2*matrix_shape[1]: 

        skip_termination = []
        changing_bot_positions = bot_positions
        bot_messages = []
        way_points = []
        duckiebot_id = []
        total_moves = 0
        total_moves_per_bot = []
        termination_tiles = []
        picked_global_coordinate = []
        i = 0
        for current_bot in bot_positions:
            different_movement_options = []
            total_tiles_list = []
            list_of_coordinates = []
            #Which termination_point it will end op on
            for termination_point in termination_positions:
                print terminations[i]
                print termination_point
                if current_bot[0] == bots and termination_point == terminations:
                    total_tiles_list.append(1000)
                    different_movement_options.append ([1000] + [1000] + [termination_point])
                    list_of_coordinates.append(1)
                    continue
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
            new_list=total_tiles_list
            new_list.sort()
            minimum_index = total_tiles_list.index(min(total_tiles_list))
            best_choice = different_movement_options[minimum_index]
            picked_global_coordinate.append(list_of_coordinates[minimum_index])   
            exact_termination = replace_termination(best_choice[2], termination_positions, termination_match_array)
            message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]
            bot_messages.append(message_to_robot)
            duckiebot_id.append(current_bot[0])
            way_points.append(best_choice[1])
            termination_tiles.append(exact_termination)
            total_moves_per_bot.append(total_tiles_list[minimum_index])
            total_moves = sum(total_moves_per_bot)            
            i+=1

    if total_moves >= 2*matrix_shape[0]*2*matrix_shape[1]:
        print("Issue")
        print("total_moves_per_bot")
        print(total_moves_per_bot)
        smallest_bot_index = total_moves_per_bot.index(min(total_moves_per_bot))
        print("smallest bot distance, index")
        print(bot_positions[smallest_bot_index],smallest_bot_index)

        skip_termination = []
        changing_bot_positions = bot_positions
        bot_messages = []
        way_points = []
        duckiebot_id = []
        total_moves = 0
        total_moves_per_bot = []
        termination_tiles = []
        picked_global_coordinate = []
        i = 0
        for current_bot in bot_positions:
            different_movement_options = []
            total_tiles_list = []
            list_of_coordinates = []
            #Which termination_point it will end op on
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
            print("current_bot")
            print(current_bot) 
            print("Total Current Robot Tiles list:")
            print(total_tiles_list)
            new_list=total_tiles_list
            new_list.sort()
            if current_bot.all()==bot_positions[smallest_bot_index].all():
                print("Bot that picks second choice")
                print(current_bot) 
                minimum_index = total_tiles_list.index(new_list[0])
            else:
                minimum_index = total_tiles_list.index(min(total_tiles_list))
            best_choice = different_movement_options[minimum_index]

            print("best_choice")
            print(best_choice)
            picked_global_coordinate.append(list_of_coordinates[minimum_index])   
            exact_termination = replace_termination(best_choice[2], termination_positions, termination_match_array)
            message_to_robot = [current_bot[0]] + [best_choice[1]] + [best_choice[0]] + [exact_termination]

            
            bot_messages.append(message_to_robot)
            duckiebot_id.append(current_bot[0])
            way_points.append(best_choice[1])
            termination_tiles.append(exact_termination)
            total_moves_per_bot.append(total_tiles_list[minimum_index])
            total_moves = sum(total_moves_per_bot)
            i += 1  """

    return bot_messages, total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles, picked_global_coordinate, terminations

def find_robot_commands(bot_position_and_orientation, optimal_movements, number_of_movements, matrix_shape, node_matrix, tile_size):
    name = bot_position_and_orientation[0]
    row = bot_position_and_orientation[1]
    column = bot_position_and_orientation[2]
    orientation = bot_position_and_orientation[3]
    transition_matrix = transition_function(matrix_shape)


    cell = row*2*matrix_shape[1] + column
    movement_commands = []
    global_coordinates = []
    x_coordinate, y_coordinate = node_to_globalcoordinates(row, column, tile_size, matrix_shape)
    global_coordinates.append([x_coordinate, y_coordinate])

    total_number_of_moments = int(number_of_movements[cell])
    for i in range (0, total_number_of_moments):
        
        best_option = optimal_movements[cell][0].astype(int)
        tiletype = node_matrix[row][column]
        transition_point= transition_matrix[best_option][tiletype]
        
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
    y = matrix_shape[0]*tile_size-(tile_size/2)*row-(tile_size/4)
    x = (tile_size/2)*column+(tile_size/4)
    return x, y

def order_optimization(all_bot_positions, duckiebot_id, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size):

    num_of_bots = len(duckiebot_id)
    available_bot_config = all_bot_positions
    all_plans = []
    total_movements = []
    way_point = []
    all_termination_tiles =[]
    
    combinations = list(permutations(available_bot_config, num_of_bots))
    
    for combination in combinations:
        array = np.asarray(combination)
        optimal_moves = -1
        plan, _, way_points, _, total_moves, _, _,terminations  = planner(array, termination_positions, termination_match_array, all_movements_matrix, matrix_shape, node_matrix, tile_size, optimal_moves)
        total_movements.append(total_moves)
        all_plans.append(plan)
        way_point.append(way_points)
        all_termination_tiles.append(terminations)
    
    min_plan_index = total_movements.index(min(total_movements))
    best_plan = all_plans[min_plan_index]
    optimal_moves = total_movements[min_plan_index]
    best_terminaton_order = all_termination_tiles[min_plan_index]
    best_configuration = np.asarray(combinations[min_plan_index])    
    print(all_termination_tiles)
    return best_configuration , optimal_moves, best_terminaton_order
