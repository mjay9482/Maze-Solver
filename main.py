#!/usr/bin/env python

"""
Multi robot simulation setup
@author: Eshant Jha, Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import sim_interface
import maze
import maze_maps
import search

def main():
    if (sim_interface.sim_init()):

        #Create three robot and setup interface for all three 
        robot1 = sim_interface.Pioneer(1)
        robot2 = sim_interface.Pioneer(2)
        robot3 = sim_interface.Pioneer(3)

        #Start simulation
        if (sim_interface.start_simulation()):
            
            current_maze = maze.Maze(1)
            #Plan for robot 1
            r1_path = search.aStarSearch(current_maze, 1, [],[])
            print("robot1_path", r1_path)
            
            #Plan for robot 2
            r2_path = search.aStarSearch(current_maze, 2,r1_path,[])
            print("robot2_path", r2_path)
            
            #Plan for robot 3
            r3_path= search.aStarSearch(current_maze,3, r1_path,r2_path)            
            print("robot3_path", r3_path)
            
            if r1_path and r2_path and r3_path:
                
                for i in range(max(len(r1_path), len(r2_path),len(r3_path))):
                    if i < len(r1_path):
                        robot1.goal_state = r1_path[i]                        
                        current_maze.map_plot_copy[r1_path[i][0]][r1_path[i][1]] = maze_maps.r1_path_id  
                    if i < len(r2_path):
                        robot2.goal_state = r2_path[i]                        
                        current_maze.map_plot_copy[r2_path[i][0]][r2_path[i][1]] = maze_maps.r2_path_id  
                    if i< len(r3_path):
                        robot3.goal_state = r3_path[i]                        
                        current_maze.map_plot_copy[r3_path[i][0]][r3_path[i][1]] = maze_maps.r3_path_id  
                    
                    print("temp positions", robot1.goal_state, robot2.goal_state, robot3.goal_state)
                    
                    #Plot the solution
                    current_maze.plot_map()
                    
                    while not robot1.robot_at_goal() or not robot2.robot_at_goal() or not robot3.robot_at_goal():
                        #Run the control loops for three robots
                        robot1.run_controller()
                        robot2.run_controller()
                        robot3.run_controller()
            else:
                print("Could not find a path")                
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #shutdown
    sim_interface.sim_shutdown()
    time.sleep(0.5)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 