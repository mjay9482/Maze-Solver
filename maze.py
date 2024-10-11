import search
import time
import copy
import maze_maps
import matplotlib
import matplotlib.pyplot as plt

class Maze:
  """
  This class outlines the structure of the maze problem
  """
  
  maze_map = []# To store map data, start and goal points
  
  # Legal moves
  # [delta_x, delta_y, description]
  five_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1], 'stop': [0, 0]}
  #eight_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1], 'stop': [0, 0], 
  #                        'upright':[-1, 1], 'upleft':[-1, -1], 'downright':[1, 1], 'downleft':[1, -1]}
  
  #Setup plot
  map_plot_copy = []
  plot_colormap_norm = matplotlib.colors.Normalize(vmin=0.0, vmax=29.0)
  fig,ax = plt.subplots(1)
  plt.axis('equal')

  def plot_map(self):
      """
      Plot
      """
      #Plotting robot 1
      start = self.getStartState(1)
      goal = self.getGoalState(1)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r1_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r1_goal_id
      #plotting robot 2
      start = self.getStartState(2)
      goal = self.getGoalState(2)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r2_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r2_goal_id
      #plotting robot 3
      
      start = self.getStartState(3)
      goal = self.getGoalState(3)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r3_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r3_goal_id
      
      plt.imshow(self.map_plot_copy, cmap=plt.cm.tab20c, norm=self.plot_colormap_norm)
      plt.show()
      
  # default constructor
  def __init__(self, id):
      """
      Sets the map as defined in file maze_maps
      """
      #Set up the map to be used
      self.maze_map = maze_maps.maps_dictionary[id]
      self.map_plot_copy = copy.deepcopy(self.maze_map.map_data)
      self.plot_map()
      return
     
  def getStartState(self, robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id == 1:
         start_state = self.maze_map.r1_start 
     elif robot_id== 2:
        start_state = self.maze_map.r2_start
     else: 
         start_state =self.maze_map.r3_start
     return start_state
 
  def getGoalState(self, robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id == 1:
         goal_state = self.maze_map.r1_goal
     elif robot_id ==2:
         goal_state = self.maze_map.r2_goal
     else:
         goal_state = self.maze_map.r3_goal
     return goal_state
    
  def isGoalState(self, robot_id, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     if state[0:2] == self.getGoalState(robot_id):
         return True
     else:
         return False

  def isObstacle(self, state):
      """
        state: Search state
     
      Returns True if and only if the state is an obstacle
      """
      if self.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
          return True
      else:
          return False
      
  def getSuccessors(self, state, dynamic_obstacle1, dynamic_obstacle2):
 
     """
       state: Seacrh state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     successors = []
     for action in self.five_neighbor_actions:
         
         #Get individual action
         del_x, del_y = self.five_neighbor_actions.get(action) 
         
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y, state[2]+1]
         new_action = action
         
         # Check for static obstacle 
         if self.isObstacle(new_successor):
             continue
         
         if dynamic_obstacle1:
             #Check for dynamic obstacle 
             if (abs(new_successor[0] - dynamic_obstacle1[0]) + abs(new_successor[1] - dynamic_obstacle1[1])) < 2:
                 continue
             #Check for dynamic obstacle 
             
         if  dynamic_obstacle2:
             if (abs(new_successor[0] - dynamic_obstacle2[0]) + abs(new_successor[1] - dynamic_obstacle2[1])) < 2:
                 continue
             
         
         #cost
         new_cost = maze_maps.free_space_cost 
             
         successors.append([new_successor, new_action, new_cost])
         
     return successors

 

# if __name__ == '__main__':
     
#     current_maze = Maze(4)
#     #Plan for robot 1
#     r1_path = search.aStarSearch(current_maze, 1, [],[])
#     print("robot1_path", r1_path)
#     #Plan for robot 2
#     r2_path = search.aStarSearch(current_maze, 2,r1_path,[])
#     print("robot2_path", r2_path)
    
#     r3_path= search.aStarSearch(current_maze,3, r1_path,r2_path) #if r1_path is considered in this map no intersection is there 
#     # intersection of r3 path r1 path
#     # intersection_list = [r3_path[i]+[i] for i in range(len(r3_path)) if r3_path[i] == r1_path[i]]
#     # print("intersection hogya" ,intersection_list)
#     # print("yha se bachna hai ",intersection_list[0][2])
#     # j=intersection_list[0][2]
#     # r2_path[j]=[intersection_list[0][0],intersection_list[0][1]]
#     # r3_path= search.aStarSearch(current_maze,3, r2_path)
    
    
#     print("robot3_path___testing",r3_path)
#     print("summing up path" ,r2_path + r1_path)
#     if r1_path and r2_path and r3_path:
        
#         for i in range(max(len(r1_path), len(r2_path),len(r3_path))):
#             if i < len(r1_path):
#                 r1_temp_pos = r1_path[i]
#             if i < len(r2_path):
#                 r2_temp_pos = r2_path[i]
#             if i< len(r3_path):
#                 r3_temp_pos = r3_path[i]
#             current_maze.map_plot_copy[r1_temp_pos[0]][r1_temp_pos[1]] = maze_maps.r1_start_id
#             current_maze.map_plot_copy[r2_temp_pos[0]][r2_temp_pos[1]] = maze_maps.r2_start_id
#             current_maze.map_plot_copy[r3_temp_pos[0]][r3_temp_pos[1]] = maze_maps.r3_start_id
            
#             #Plot the solution
#             current_maze.plot_map()
            
#             time.sleep(1.9)
#     else:
#         print("Could not find a path")
       
       
# if __name__ == '__main__':
              
#     current_maze = maze.Maze(1)
#     #Plan for robot 1
#     r1_path = search.aStarSearch(current_maze, 1, [],[])
#     print("robot1_path", r1_path)
    
#     #Plan for robot 2
#     r2_path = search.aStarSearch(current_maze, 2,r1_path,[])
#     print("robot2_path", r2_path)
    
#     #Plan for robot 3
#     r3_path= search.aStarSearch(current_maze,3, r1_path,r2_path)            
#     print("robot3_path", r3_path)
    
#     if r1_path and r2_path and r3_path:
        
#         for i in range(max(len(r1_path), len(r2_path),len(r3_path))):
#             if i < len(r1_path):
#                 robot1.goal_state = r1_path[i]                        
#                 current_maze.map_plot_copy[r1_path[i][0]][r1_path[i][1]] = maze_maps.r1_path_id  
#             if i < len(r2_path):
#                 robot2.goal_state = r2_path[i]                        
#                 current_maze.map_plot_copy[r2_path[i][0]][r2_path[i][1]] = maze_maps.r2_path_id  
#             if i< len(r3_path):
#                 robot3.goal_state = r3_path[i]                        
#                 current_maze.map_plot_copy[r3_path[i][0]][r3_path[i][1]] = maze_maps.r3_path_id  
            
#             print("temp positions", robot1.goal_state, robot2.goal_state, robot3.goal_state)
            
#             #Plot the solution
#             current_maze.plot_map()        
#             time.sleep(1.9)
#     else:
#         print("Could not find a path")