#!/usr/bin/env python
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
        print(transition_matrix)
        return transition_matrix