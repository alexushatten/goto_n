#!/usr/bin/env python
import numpy as np

def transition_function(matrix_shape):
        north= -2*matrix_shape[1]
        south= 2*matrix_shape[1]
        west=-1
        east=1
        #each column is from 0 to 20    
        #each row is 0=north 1=east 2=west 3=south
        transition_matrix=[\
        [0, north,  0,   0,   0,   0,  2*north+east, 2*north,     0,     north,   2*north,  0,           0,     0,             0,       north, 2*north+east,  0,       north,   2*north+east, 2*north ],\
        [0,   0,    0,  east, 0,   0,     0,           0,    south+2*east, 0,       east,   0,        2*east,  east,      south+2*east,   0,      2*east, south+2*east,  0,       2*east,      east],\
        [0,   0,    0,   0, west, west,   0,     north+2*west,    0,       0,         0,  2*west,        0,  north+2*west,   west,     2*west,       0,       west,    2*west,       0,      north+2*west],\
        [0,   0,  south, 0,   0, 2*south,south,        0,      2*south, 2*south+west, 0, 2*south+west, south,   0,              0,        0,         0,      2*south, 2*south+west, south,      0],\
        [0,   0,    0,   0,   0,   0,     0,           0,          0,      0,         0,     0,          0,     0,              0,        0,         0,        0,         0,          0,        0]]
        return transition_matrix


def go_matrix(matrix_shape, node_matrix, transition_matrix, direction):
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1]
    movement_matrix = np.zeros((matrix_size,matrix_size))
    nodes = np.array(node_matrix).flatten()
    if direction =="north":j=0
    elif direction =="east":j=1
    elif direction =="west":j=2
    else: j=3
    for i in range(0,matrix_size):
        transition_point=transition_matrix[j][nodes[i]]
        if transition_point != 0:
            movement_matrix[i,i+transition_point] = 1                        
    return movement_matrix
