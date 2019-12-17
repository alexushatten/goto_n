#!/usr/bin/env python
import numpy as np

'''
The functions in this file are used to extract the relevant map information from any given 
yaml file (as long as it is in the format of the other map yaml files in duckietown world)
in order to use it in our dynnamic programming planning algorithm
'''

def extract_tile_matrix(map_data):
    """
    Extracts the tile specific information for the map that the goto-n should be performed on.

    The input of the funciton is the map_data matrix. As can be seen in the goto_n.py file, the 
    map_data matrix uses the load_map function (from duckietown_utils) in order to extract the specific 
    tile type, matrix sixe and tile size from the given map yaml file.

    Args:
        map_data (matrix): The map_data matrix is the matrix created when the load_map function from
        duckietown-utils is used to read in the map yaml file

    Returns:
        The tile_matrix: A matrix consisting of the sequential tile types for the map (in strings)
        Tile_matrix.shape: The size of the inputted map
        tile_size: The size of the tiles used in the map
    """

    tile_matrix = []
    for tile in map_data["tiles"]:
        tile_matrix.append(tile)
    tile_matrix = np.array(tile_matrix)
    
    return tile_matrix, tile_matrix.shape, map_data["tile_size"]


def encode_map_structure(tile_matrix, matrix_shape):
    """
    Encodes a numerical value that represents the tile type found in the map

    Our planning algorithm makes decisions (has allowable movements) depending on the 
    tile type that the duckiebot is currently on. This function takes the tile_matrix (matrix 
    composed of the names of the respective tiles found in the map) and replaces the string with
    an integer value representing the tile type. 

    Args:
        tile_matrix: The matrix containing the names of the tiles found in the Robotarium map
        matrix_shape: The shape of the above matrix

    Returns:
       Node_matrix: A matrix, the same size as the tiles on the map, that represents the 
       specific tiles with integers 
    """

    row = tile_matrix.shape[0]
    column = tile_matrix.shape[1]

    node_matrix = np.zeros((matrix_shape[0], matrix_shape[1]))
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


def extend_matrix(node_matrix, matrix_shape):
    """
    Extends the node_matrix created above to increase the accuracy/resolution of the planner

    This function takes the node_matrix, the matrix that encodes specific actions based on the current
    tile type, and replaces it with a more granular representation of the matrix. As a result, 
    each tile has four distinct states in which the duckiebot can act differently instead of just one. 

    In the case of the K31 lab, this function is used to turn the (6,6) matrix into a more granular (12,12)
    matrix. 

    Args:
        node_matrix: The matrix containing the integers of the tiles found in the Robotarium map
        matrix_shape: The shape of the above matrix

    Returns:
       Extended_matrix: A matrix that contains inegers in the respective places of the map that
       indicate the set of possible actions for the robot. 
    """

    extended_matrix=np.zeros((2*matrix_shape[0],2*matrix_shape[1]))
    for i in range(0,matrix_shape[0]):
        for j in range(0, matrix_shape[1]):
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
