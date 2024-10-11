# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 12:16:30 2023

@author: Bijo Sebastian
"""

#Definitions based on color map
r1_start_id = 1
r1_path_id = 2
r1_goal_id = 1
r2_start_id = 12
r2_path_id = 13
r2_goal_id = 12
r3_start_id = 8
r3_path_id = 10
r3_goal_id = 8
obstacle_id = 16
free_space_id = 3
free_space_cost = 3


class Maps:
    """
    This class outlines the structure of the maps
    """    
    map_data = []
    start = []
    goal = []

#Maze maps
map_1 = Maps()
map_1.map_data = [[16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16],
                  [16,  3,  3,  3,  3,  3,  3,  3,  3,  3, 16],
                  [16,  3, 16,  3, 16,  3, 16,  3, 16,  3, 16],
                  [16,  3,  3,  3,  3,  3,  3,  3,  3,  3, 16],
                  [16,  3, 16,  3, 16,  3, 16,  3, 16,  3, 16],
                  [16,  3,  3,  3,  3,  3,  3,  3,  3,  3, 16],
                  [16,  3, 16,  3, 16,  3, 16,  3, 16,  3, 16],
                  [16,  3,  3,  3,  3,  3,  3,  3,  3,  3, 16],
                  [16,  3, 16,  3, 16,  3, 16,  3, 16,  3, 16],
                  [16,  3,  3,  3,  3,  3,  3,  3,  3,  3, 16],
                  [16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16]]

map_1.r1_start = [2, 1]
map_1.r2_start = [6, 1]
map_1.r3_start = [9, 8]
map_1.r1_goal =  [6, 5] 
map_1.r2_goal =  [2, 5]                            
map_1.r3_goal =  [4, 5]




maps_dictionary = {1:map_1}


 
