#!/usr/bin/env python
import numpy as np
import os
import rospy
import yaml

from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from duckietown_msgs.msg import BoolStamped
from duckietown import DTROS
from visualization_msgs.msg import MarkerArray
from duckietown_utils import load_map

#NEW FUNCTIONS
from goto_n_transition_matrix import *
from goto_n_extract_map_info import *
from goto_n_localize import *
from goto_n_planner import *


class GoToNNode(DTROS):
    """
    GoToNNode

    This node implements the path planning on the server side. It is the global path planner for
    all the duckiebots running the Goto_N_Duckiebot node. 

    Args:
        GotoNNode(self)

    Subscriber:
        /cslam_markers (:obj:'MarkerArray'): Subscribes to the position and orientation from the online 
            localization. 

    Publisher:
        /goto_n/termination_commands (:obj:`Float32MultiArray`): The global x/y positions of the termination
            state that the given duckiebot is driving towards
        /autobot{}/movement_commands (:obj'Int32MultiArray'): These are the intersection movement commmands
            that are sent to the respective duckiebots
        /autobot0{}/arrival_msg (:obj:'duckietown_msgs.msg.BoolStamped'): Boolean that activates the final 
            accuracy node once the last command has been completed
        /autobot0{}/plan_visualization (:obj:'nav_msgs.msg.Path'): The path coordinates sent out to the RVIZ
            subscriber. 
        /'+watchtower+'/'+"requestImage" (:obj:'Bool'): A boolean that requests images from the watchtower to 
            localize the duckiebots. 
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(GoToNNode, self).__init__(node_name=node_name)

        # Set up map
        self.map_name = 'ethz_amod_lab_k31'
        self.map_filename = '/code/catkin_ws/src/goto_n/packages/goto_n/maps/{}.yaml'.format(self.map_name)
        self.map_data = load_map(self.map_filename)
        self.map_matrix, self.matrix_shape, self.tile_size = extract_tile_matrix(self.map_data)
        small_node_matrix = encode_map_structure(self.map_matrix, self.matrix_shape)
        self.node_matrix = extend_matrix(small_node_matrix, self.matrix_shape)

        # Create movement_matricies
        self.transition_matrix=transition_function(self.matrix_shape)
        self.go_north=go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"north")
        self.go_east= go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"east")
        self.go_south=go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"south")
        self.go_west= go_matrix(self.matrix_shape, self.node_matrix, self.transition_matrix,"west")
        self.go_nowhere = np.eye(2*self.matrix_shape[0]*2*self.matrix_shape[1])
        self.all_movements_matrix = np.concatenate((self.go_north, self.go_east, self.go_west, self.go_south, self.go_nowhere), 1)
        
        #Set up message process
        self.message_recieved = False
        self.message_sent = False

        #Initialize termination positions  [Row, Column, Direction]
        self.all_termination_positions = rospy.get_param('~termination_positions_list')
        self.termination = self.extract_termination_tile()
        self.all_termination_positions_array = np.array(self.all_termination_positions)
        self.termination_array = np.array(self.termination)
        self.termination_match_array=np.concatenate((self.termination_array,self.all_termination_positions_array),axis=1)

        # Start Publisher for each duckiebot
        self.movement_cmd_pub=[]
        self.arrival_msg_sub=[]
        self.plan_viz_pub=[]
        self.termination_commands = rospy.Publisher('/goto_n/termination_commands', Float32MultiArray, queue_size=10)

        # Initialize duckiebots 
        self.autobot_list = rospy.get_param('~duckiebots_list')
        self.arrival_msg_list = []
        for autobot in self.autobot_list:
            if autobot < 10:
                self.movement_cmd_pub.append(rospy.Publisher('/autobot0{}/movement_commands'.format(autobot), Int32MultiArray, queue_size=10))
                self.arrival_msg_sub.append(rospy.Subscriber('/autobot0{}/arrival_msg'.format(autobot), BoolStamped, self.arrival_callback))
                self.plan_viz_pub.append(rospy.Publisher('/autobot0{}/plan_visualization'.format(autobot), Path, queue_size=10))
            else:
                self.movement_cmd_pub.append(rospy.Publisher('/autobot{}/movement_commands'.format(autobot), Int32MultiArray, queue_size=10))
                self.arrival_msg_sub.append(rospy.Subscriber('/autobot{}/arrival_msg'.format(autobot), BoolStamped, self.arrival_callback))
                self.plan_viz_pub.append(rospy.Publisher('/autobot{}/plan_visualization'.format(autobot), Path, queue_size=10))

        #Ensure that the planner is correctly set up
        self.proper_initialization()

        print('The Duckiebots given in the list are: {} \n'.format(self.autobot_list))

        #Start localization subscriber
        self.localization_subscriber = rospy.Subscriber("/cslam_markers", MarkerArray, self.callback)

        # Initialize watchtower request image message
        self.watchtowers_list = rospy.get_param('~watchtowers_list')
        self.image_request=[]
        for watchtower in self.watchtowers_list:
            self.image_request.append(rospy.Publisher('/'+watchtower+'/'+"requestImage",Bool,queue_size=1))

        # Request image from watchtower
        self.request_watchtower_image()

    def proper_initialization(self):
        """
        This  function determines if the initialization parameters are met. I.e. if there 
        are an equal number of duckiebots and termination states present.
        """

        #checks if the initialization parameters are met
        if len(self.autobot_list) == len(self.all_termination_positions):
            print('\nThere are an equal number of autobots and termination states. Starting Goto_n Node! \n')
        elif len(self.all_termination_positions) > len(self.autobot_list):
            print('\nMissing Autobots. Plese retry localization. \n')
        else: 
            print('\nPlease enter another Termination State \n')    
        
    def request_watchtower_image(self):
        """
        This function requests images from the watchtower in order to perform the localization of the duckiebots 
        """

        #requests an image from watchtowers in town
        msg_sended = Bool()
        rospy.sleep(1)
        msg_sended.data = True 
        for i in range(0,len(self.watchtowers_list)):
            self.image_request[i].publish(msg_sended)

    def extract_autobot_data(self, marker):
        """
        This function takes the Marker Array from the localization system and extracts the relevant
        information that will then be used later in path planning. 

        Args:
            Marker (:obj:'Visualization_msgs.msg.MarkerArray'): The marker array contains the position
                and orientation information from the online localization.

        Returns:
            all_bot_positions (:obj:'Int32MultiArray'): Creates an array with all bot positions and orienations
                in the node notation
            bot_ids (:obj:'Int32Array'): Array containing all the duckiebot ids
        """

        #Initialize the necessary arrays
        all_bot_positions = []
        bot_ids = []

        for bots in marker: 
            if bots.ns == "duckiebots":

                #Set duckiebots position and orientation
                duckiebot_id = bots.id

                if duckiebot_id in self.autobot_list:
                    duckiebot_x = bots.pose.position.x
                    duckiebot_y = bots.pose.position.y
                    duckiebot_orientation= quat_to_compass(bots.pose.orientation)
                    duckie_compass_notation = find_compass_notation(duckiebot_orientation)
                    
                    #Find duckiebots tile column and row
                    duckiebot_row, duckiebot_column = find_current_tile(duckiebot_x, duckiebot_y, duckie_compass_notation, self.matrix_shape, self.tile_size, self.node_matrix)
                    
                    all_bot_positions.append([duckiebot_id, duckiebot_row, duckiebot_column, duckie_compass_notation])
                    bot_ids.append(duckiebot_id)
                    self.message_recieved = True

        return all_bot_positions, bot_ids

    def extract_termination_tile(self):
        """
        This function takes the global positions of the termination states and converts them into
        node notation and orientation to be used in the path planning.

        Returns:
            termination_tile_positions (:obj:'Int32MultiArray'): An array that contains all the termination 
                position in tile notation. 
        """

        termination_positions = self.all_termination_positions
        termination_tile_positions = []        

        for termination_position in termination_positions:
            pose_x = termination_position[0]
            pose_y = termination_position[1]
            orientation = termination_position[2]
            termination_compass_notation = find_compass_notation (orientation)
            termination_row, termination_column = find_current_tile(pose_x, pose_y, termination_compass_notation, self.matrix_shape, self.tile_size, self.node_matrix)
            termination = [termination_row, termination_column, termination_compass_notation]
            termination_tile_positions.append(termination)

        return termination_tile_positions

    def arrival_callback(self, arrival_msg):
        """
        This function lets the server know if it has arrived. If one of the duckiebots returns this 
        as false, this function replans the path for the given duckiebot in order to ensure all duckiebots
        arrive at the desired location. 

        Args:
            arrival_msg (:obj:'duckietown_msgs.msg.BoolStamped'): The marker array contains the position
            and orientation information of all the duckiebots on the map. 
        """

        self.arrival_msg_list.append(arrival_msg.data)
        all_robots_in_position = False
        if len(self.arrival_msg_list) == len(self.autobot_list):
            for message in self.arrival_msg_list:
                if message == False:
                    self.message_recieved = False
                    print("Replanning for robots")
                    self.arrival_msg_list = []
                    all_robots_in_position = True
                    return
            if all_robots_in_position:
                print("all robots back in position")

    def callback(self, markerarray):
        """
        The callback function receives an input from the localization (one time input). It then
        performs the Goto-N functionality by planning the paths for all duckiebot and publishing the 
        movement commands to the respective duckiebots. 
        Args:
            Markerarray (:obj:'Visualization_msgs.msg.MarkerArray'): The marker array contains the position
                and orientation information of the duckiebots from the online localization.

        Publisher:
        /goto_n/termination_commands (:obj:`Float32MultiArray`): The global x/y positions of the termination
            state that the given duckiebot is driving towards
        /autobot{}/movement_commands (:obj'Int32MultiArray'): These are the intersection movement commmands
            that are sent to the respective duckiebots
        /autobot0{}/plan_visualization (:obj:'nav_msgs.msg.Path'): The path coordinates sent out to the RVIZ
            subscriber.
        """

        #initialize all necessary arrays
        marker = markerarray.markers
        all_bot_positions = []

        if self.message_recieved == False:
            #check if it finds the duckiebots and pose

            all_bot_positions, bot_ids = self.extract_autobot_data(marker)
            print('Received Message from the Online Localization. Starting Planning Algorithm: \n')

            #determine the optimal order and run the planning algorithm
            best_start_config = order_optimization(all_bot_positions, bot_ids, self.termination, \
            self.termination_match_array, self.all_movements_matrix, self.matrix_shape, self.node_matrix, self.tile_size)
            plan,total_moves_per_bot, way_points, duckiebot_id, total_moves, termination_tiles, global_coordinates = planner(best_start_config, self.termination, \
            self.termination_match_array, self.all_movements_matrix, self.matrix_shape, self.node_matrix, self.tile_size)

            print("Duckiebot id and Termination tiles")
            print(duckiebot_id)
            print(termination_tiles)

            #this is the code for multiple autobot
            for i in range(0, len(bot_ids)):

                #determine if all autobots have been located
                if len(bot_ids) !=len(self.autobot_list):
                    print("Could not locate all duckiebots in the list")
                    return

                #assign the waypoint commands to the right duckiebot
                bot_indx= duckiebot_id.index(self.autobot_list[i])
                print(bot_indx)
                duckiebot = self.autobot_list[bot_indx]
                duckiebot_float = float(duckiebot_id[bot_indx])
                message = way_points[bot_indx]
                duckiebot_coordinates =  global_coordinates[bot_indx]
                termination = termination_tiles[bot_indx]
                termination_message = [duckiebot_float] + termination 
                pum_msg = Int32MultiArray()   
                print('The starting Positions of all bots are: {} \n'.format(all_bot_positions))
                print('The waypoint commands sent out to the duckiebot {} are {}. \n'.format(duckiebot_id[bot_indx], message))
                print('Waypoint Commands sent out to the respective Duckiebots! \n')
                
                #Publish the waypoint commands the right duckiebot. 
                rospy.sleep(1)
                pum_msg = Int32MultiArray(data=message)
                self.movement_cmd_pub[i].publish(pum_msg)

                if self.message_sent == False:
                    term_msg = Float32MultiArray()
                    rospy.sleep(1)
                    term_msg = Float32MultiArray(data=termination_message)
                    self.termination_commands.publish(term_msg)
                
                #Publish the visualization data to RVIZ
                path = Path()
                path.header.frame_id = "map"
                for coordinate in duckiebot_coordinates:
                    cur_pose = PoseStamped()

                    cur_pose.header.stamp = rospy.Time.now()
                    cur_pose.pose.position.x= coordinate[0]
                    cur_pose.pose.position.y = coordinate[1]
                    cur_pose.pose.position.z = 0.1
                    cur_pose.pose.orientation.w = 1

                    path.poses.append(cur_pose)
                self.plan_viz_pub[i].publish(path)
                
            self.message_sent = True

if __name__ == '__main__':
    # Initialize the node
    goto_node = GoToNNode(node_name='goto_n')
    # Keep it spinning to keep the node alive
    rospy.spin()