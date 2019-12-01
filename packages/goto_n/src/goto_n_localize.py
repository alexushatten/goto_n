#!/usr/bin/env python
import math

def quat_to_compass( q):
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

def find_compass_notation(deg):
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


def find_current_tile(pose_x, pose_y, orientation, matrix_shape, tile_size, node_matrix):
    number_of_rows = 2*matrix_shape[0]
    number_of_columns = 2*matrix_shape[1]
    #Get the column
    tile_column = int(round(((tile_size/4) + 2*pose_x/tile_size) - 1))
    if tile_column < 0:
        tile_column = 0
    if tile_column > number_of_columns - 1:
        tile_column = number_of_columns - 1

    #Get the row
    tile_row = int((number_of_rows - 1) - round(((tile_size/4) + 2*pose_y/tile_size - 1)))
    if tile_row < 0:
        tile_row = 0
    elif tile_row > number_of_rows - 1:
        tile_row = number_of_rows - 1
    current_tile_type=node_matrix[tile_row, tile_column]
    next_column = int(round(((tile_size/4) + 2*pose_x/tile_size)))
    previous_column = int(round(((tile_size/4) + 2*pose_x/tile_size-2)))
    next_row = int(number_of_rows - round(((tile_size/4) + 2*pose_y/tile_size - 1)))
    previous_row = int(number_of_rows -2 - round(((tile_size/4) + 2*pose_y/tile_size - 1)))

    #If tiletype does not match orientation look at neighbouring tiles
    if previous_column < 0:
        western_tile_type= - 1
    else:
        western_tile_type=node_matrix[tile_row, previous_column]
    if previous_row < 0:
        northern_tile_type= - 1
    else:
        northern_tile_type=node_matrix[previous_row, tile_column]
    if next_row > number_of_rows - 1:
        southern_tile_type= - 1
    else:
        southern_tile_type=node_matrix[next_row, tile_column]
    if next_column > number_of_columns - 1:
        eastern_tile_type = - 1
    else:
        eastern_tile_type=node_matrix[tile_row, next_column]

    if current_tile_type < 5:
        if orientation == 0 and current_tile_type != 1:
            if western_tile_type == 1:
                tile_column=previous_column
            elif eastern_tile_type == 1:
                tile_column=next_column
            elif northern_tile_type == 1:
                tile_row=previous_row
            elif southern_tile_type == 1:
                tile_row=next_row
        elif orientation == 1 and current_tile_type != 3:
            if northern_tile_type == 3:
                tile_row=previous_row
            elif southern_tile_type == 3:
                tile_row=next_row
            elif western_tile_type == 3:
                tile_column=previous_column
            elif eastern_tile_type == 3:
                tile_column=next_column
        elif orientation == 2 and current_tile_type != 4:
            if northern_tile_type == 4:
                tile_row=previous_row
            elif southern_tile_type == 4:
                tile_row=next_row
            elif western_tile_type == 4:
                tile_column=previous_column
            elif eastern_tile_type == 4:
                tile_column=next_column
        elif orientation == 3 and current_tile_type != 2:
            if western_tile_type == 2:
                tile_column=previous_column
            elif eastern_tile_type == 2:
                tile_column=next_column
            elif northern_tile_type == 2:
                tile_row=previous_row
            elif southern_tile_type == 2:
                tile_row=next_row
    return tile_row, tile_column
    