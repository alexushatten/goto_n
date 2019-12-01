import numpy as np

def cost_matrix(matrix_shape, all_movements_matrix, termination_point, all_bot_positions, current_bot_position):
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1]
    
    #Init the cost matrix
    cost_mat=np.matmul(all_movements_matrix[0:matrix_size,0:matrix_size],np.ones((matrix_size,1)))
    cost_mat=np.append(cost_mat,np.matmul(all_movements_matrix[0:matrix_size,matrix_size:2*matrix_size],np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(all_movements_matrix[0:matrix_size,2*matrix_size:3*matrix_size],np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(all_movements_matrix[0:matrix_size,3*matrix_size:4*matrix_size],np.ones((matrix_size,1))),1)
    cost_mat=np.append(cost_mat,np.matmul(all_movements_matrix[0:matrix_size,4*matrix_size:5*matrix_size],1000*np.ones((matrix_size,1))),1)
    cost_mat[cost_mat < 0.1]=float('inf')

    #Add termination_point
    termination_row = termination_point[0]
    termination_column = termination_point[1]
    termination_direction = termination_point[2]
    #Add 0 cost to desired location
    cell = termination_row*2*matrix_shape[1] + termination_column
    cost_mat[cell,4] = 0

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

def value_iteration(matrix_shape, all_movements_matrix, cost_mat):
    matrix_size = 2*matrix_shape[0]*2*matrix_shape[1] #Size of all the number nodes

    number_of_movements = np.zeros((matrix_size,1)) 
    updated_number_of_moves = np.zeros((matrix_size,1))
    optimal_movements =np.zeros((matrix_size,1))
    movements_per_command=np.zeros((5,1))
    V_1=np.zeros((matrix_size,1))
    V_1[0]=float('inf')
   
    iterate = True
    iteration_value=0

    while iterate == True:
        iteration_value+=1
        for i in range(0, matrix_size):
            for k in range(0,5):
                matrix_for_all_commands=np.matmul(all_movements_matrix[:,matrix_size*k:matrix_size*(k+1)],number_of_movements)
                movements_per_command[k]=cost_mat[i,k] + matrix_for_all_commands[i]
            updated_number_of_moves[i]=np.amin(movements_per_command)
            optimal_movements[i]=np.argmin(movements_per_command)
        v_test=updated_number_of_moves
        v_test[v_test > matrix_size] = matrix_size
        V_1[iteration_value]=np.matmul(np.ones((1,matrix_size)),v_test)
        number_of_movements=updated_number_of_moves
        if V_1[iteration_value]-V_1[iteration_value-1]==0:
            iterate = False
        elif iteration_value== matrix_size:
            iterate = False

    number_of_movements[number_of_movements > 1000]=float('inf')

    return optimal_movements , number_of_movements.astype(int)
