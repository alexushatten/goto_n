#!/usr/bin/env python
import numpy as np

'''
The functions in this file are used to create the transition functions and the go_matrix
'''

def transition_function(matrix_shape):
    """
    This function takes in the shape of the matrix and creates the transition matrix used int he go_matrix
    and the cost_matrix. For each type of tile created, the transition matrix defines the set of connected nodes 
    that can be reached from the current node.

    Args:
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix

    Returns:
        transition_matrix (:obj:'Matrix'): A populated transition matrix that defines the set of possible transitions
        for each tile type (type of node)
    
    Additional
        The multiplication by a factor of 2 represents the increase in granularity of the node_matrix
    
    """

    #north means that the duckiebot moves to the previous row in the node matrix
    #south means the duckiebot  moves to the next row in the node matrix
    #similar for east/west -> previous/next elements
    north= -2*matrix_shape[1]
    south= 2*matrix_shape[1]
    west=-1
    east=1
    
    #definition of transition matrix
    transition_matrix=[\
    [0, north,  0,   0,   0,   0,  2*north+east, 2*north,     0,     north,   2*north,  0,           0,     0,             0,       north, 2*north+east,  0,       north,   2*north+east, 2*north ],\
    [0,   0,    0,  east, 0,   0,     0,           0,    south+2*east, 0,       east,   0,        2*east,  east,      south+2*east,   0,      2*east, south+2*east,  0,       2*east,      east],\
    [0,   0,    0,   0, west, west,   0,     north+2*west,    0,       0,         0,  2*west,        0,  north+2*west,   west,     2*west,       0,       west,    2*west,       0,      north+2*west],\
    [0,   0,  south, 0,   0, 2*south,south,        0,      2*south, 2*south+west, 0, 2*south+west, south,   0,              0,        0,         0,      2*south, 2*south+west, south,      0],\
    [0,   0,    0,   0,   0,   0,     0,           0,          0,      0,         0,     0,          0,     0,              0,        0,         0,        0,         0,          0,        0]]
    
    return transition_matrix


def go_matrix(matrix_shape, node_matrix, transition_matrix, direction):
    """
    The goal of the go_matrix function is to define a movement_matrix that determines the possible
    set of geometric moves (N/E/S/W) from each location on the map. 

    Args:
        matrix_shape (:obj:'Tuple'): Size of the encoded matrix
        node_matrix (:obj:'Matrix'): Matrix that has the tile type encoded into it
        transition_matrix (:obj:'Matrix'): Matrix that defines all the possible tiles that can be reached
            for each tile type
        directon (:obj:'Int'): Direction that the duckiebot is currently facing

    Returns:
        movement_matrix (:obj:'Matrix'): A populated movement_matrix that defines all the moves a duckiebot
            can undertake if it in a certain geographical location. 
    
    Additional
        The multiplication by a factor of 2 represents the increase in granularity of the node_matrix
    
    """
    #create the required matrix size
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1]
    movement_matrix = np.zeros((matrix_size,matrix_size))
    
    #define the nodes
    nodes = np.array(node_matrix).flatten()
    
    #encode movement directions for specific node
    if direction =="north":j=0
    elif direction =="east":j=1
    elif direction =="west":j=2
    else: j=3
    for i in range(0,matrix_size):
        transition_point=transition_matrix[j][nodes[i]]
        if transition_point != 0:
            movement_matrix[i,i+transition_point] = 1                        
    
    return movement_matrix