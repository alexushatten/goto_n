#!/usr/bin/env python
import numpy as np

def extract_tile_matrix(map_data):
    tile_matrix = []
    for tile in map_data["tiles"]:
        tile_matrix.append(tile)
    tile_matrix = np.array(tile_matrix)
    return tile_matrix, tile_matrix.shape, map_data["tile_size"]


def encode_map_structure(tile_matrix, matrix_shape):
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
