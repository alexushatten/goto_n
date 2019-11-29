import numpy as np

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

def cost_matrix(matrix_shape, go_north, go_east, go_west, go_south, go_nowhere, termination_point, all_bot_positions, current_bot_position):
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1]
    cost_mat=np.matmul(go_north,np.ones((matrix_size,1)))
    cost_mat=np.append(cost_mat,np.matmul(go_east,np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(go_west,np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(go_south,np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(go_nowhere,1000*np.ones((matrix_size,1))),1)
    cost_mat[cost_mat < 0.1]=float('inf')

    #Add termination_point
    termination_row = termination_point[0]
    termination_column = termination_point[1]
    termination_direction = termination_point[2]
    #Add 0 cost to desired location
    cell = termination_row*2*matrix_shape[1] + termination_column
    cost_mat[cell,4]=0

    current_name = current_bot_position[0]
    for other_bot_position in all_bot_positions:

        other_name = other_bot_position[0]
        #Skip the bot you are currently calculating the map for
        if other_name == current_name:
            continue

        other_init_row = other_bot_position[1]
        other_init_column = other_bot_position[2]
        other_init_direction = other_bot_position[3]
        other_cell = other_init_row*2*matrix_shape[1] + other_init_column
        cost_mat[other_cell,other_init_direction] = float('inf')

    return cost_mat

def value_iteration(matrix_shape, go_north, go_east, go_west, go_south, go_nowhere, cost_mat):
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1]
    V_matrix = np.zeros((matrix_size,1))
    V_new_matrix = np.zeros((matrix_size,1))
    I_matrix =np.zeros((matrix_size,1))
    V_temporary_matrix=100*np.ones((matrix_size,1))
    V_amound_of_inputs=np.zeros((5,1))
    iteration_value=0
    all_movements_matrix = np.append(go_north, go_east, 1)
    all_movements_matrix = np.append(all_movements_matrix, go_west, 1)
    all_movements_matrix = np.append(all_movements_matrix, go_south, 1)
    all_movements_matrix = np.append(all_movements_matrix, go_nowhere, 1)
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
        if iteration_value==60:
            iterate = False
    Optimal_movements=I_matrix
    Number_Of_Movements=V_matrix
    Number_Of_Movements[Number_Of_Movements > 1000]=float('inf')

    return Optimal_movements , Number_Of_Movements.astype(int)
