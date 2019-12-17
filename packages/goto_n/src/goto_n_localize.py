#!/usr/bin/env python
import math

"""
The functions in this file are used to transform the data/topics received from the localization
publisher and turn them into the form that is necessary to run our planning algorithm. 
"""

def quat_to_compass(q):
    """
    This function takes the quaterion orientation from the localization system and turns it into
    compass orientation in degrees. 

    Args:
        q (Float): Quaternion information received from the localization system

    Returns:
        deg (Int): Orientation of the Duckiebot in Degrees, with 0 being North  
    """
    
    #transform coordinates
    sqw = q.w * q.w
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z

    #compute the normal
    normal = math.sqrt(sqw + sqx + sqy + sqz)
    
    #determine poles
    pole_result = (q.x * q.z) + (q.y * q.w)
    
    #define pi
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
    """
    Encodes a numerical value that represents the tile type found in the map

    Args:
        deg (Integer): The orientation of the Duckiebot in degrees (North is 0)

    Returns:
       compass (Integer): Integer encoded value of the general orientation of the duckiebot. This is used in 
       the planning algorithm
    
    Additional:
        We assign a singular value to check if the duckiebot is correctly orientated in the lane. If this 
        is the case, small deviations in orientation are negligible as the duckiebot will move using 
        indefinite navigation. 
    """
    #initialize compass
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
    """
    This function finds the current positional tile of the duckiebot, represented in our encoded 
    notation. 

    The function takes in data received from the localization system and determines the tile the 
    duckiebot is currently located on in our node representation of the map.  

    Args:
        pose_x (Float): Float of the global x-coordinate of the duckiebot 
        pose_y (Float): Float of the global y-coordinate of the duckiebot 
        orientation (Float): Orientation of the duckiebot as received from the localization system
        matrix_shape (Tuple): The size of the encoded map matrix 
        tile_size (Float): The size of the tiles
        node_matrix (Matrix): Matrix representing the types of tiles in the map

    Returns:
       Tile_row (Int): The row in the extended matrix that the duckiebot is current on 
       Tile_column (Int): The row in the extended matrix that the duckiebot is current on
    
    Additional:
        This function was created before the increase in resolution that we added for the specific map
        As a result, it multiplies the previous matrix by 2 (in both length and width) in order to 
        represent the more accurate tile matrix. 

        Error Management: The function also implements some error management in order to add robustness
            If the tile type does not match the orientation that is expected, it might be possible that the 
            duckiebot is close to the boundary between tiles/very close to being off the lane. As a result, 
            the neighboring tiles are checked and the orientation is corrected to make it coherent with the 
            neighboring tiles. 
    """
    
    #define number of rows and columns
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
    
    #define the current tile
    current_tile_type=node_matrix[tile_row, tile_column]
    
    #determine the columns and rows around the duckiebot tile
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

    #checks if the orientation of the current tile is as expected according to the tile type
    #implemented in order to increase robustness for cases where the robot is on the verge of two
    #tiles and localized on the wrong tile. 

    if current_tile_type < 5:
        if current_tile_type == 0 and western_tile_type == 0 and eastern_tile_type == 0 and northern_tile_type == 0 and southern_tile_type == 0:
            print("Out of bounds, call city rescue !")
        elif orientation == 0 and current_tile_type != 1:
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