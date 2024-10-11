import operator

def heuristic_1(problem, robot_id, state):
    """
    Euclidean distance
    """
    xs = state[0]
    ys = state[1]
    (xg, yg) = problem.getGoalState(robot_id)

    # Euclidean heuristic
    h = ((xs - xg)**2 + (ys - yg)**2)**0.5
    return h
def heuristic_2(problem, robot_id, state):
    """
    Euclidean distance
    """
    xs = state[0]
    ys = state[1]
    (xg, yg) = problem.getGoalState(robot_id)

    # Euclidean heuristic
    h = (abs(xs - xg)+ abs(ys - yg))
    return h


def aStarSearch(problem, robot_id, dynamic_obstacle1,dynamic_obstacle2):
    fringe = [] # to store the nodes that are to be explored 
    explored = [] # to store the nodes that are yet to be explored
    
    # add the start state to the fringe with a cost of 0
    start_state = problem.getStartState(robot_id)
    start_state.append(0)
    start_cost = 0
    start_heuristic = heuristic_2(problem, robot_id, start_state)
    fringe.append([start_state, [[start_state[0], start_state[1]]], start_cost, start_heuristic + start_cost])
    
    while len(fringe) > 0:
        
        #Sort fringe in order of least cost/level
        fringe = sorted(fringe, key = operator.itemgetter(3))
        
        #Pop least cost node and add to explored list
        current_node = fringe.pop(0)
        
        explored.append(current_node[0])# only the state needs to be added to explored list
        
        # check if the current state is the goal state
        if problem.isGoalState(robot_id, current_node[0]):
            return (current_node[1])
            
        # expand the current state by performing all possible actions
        current_time = current_node[0][2]
        if len(dynamic_obstacle1) < ( current_time + 2) :
            successors = problem.getSuccessors(current_node[0],[],[])
        elif len(dynamic_obstacle2) < ( current_time + 2) :
            successors = problem.getSuccessors(current_node[0],[],dynamic_obstacle1[current_time +1])
        else:
            successors = problem.getSuccessors(current_node[0], dynamic_obstacle1[current_time + 1], dynamic_obstacle2[current_time + 1])
       
        for successor_state, successor_action, successor_cost in successors:
            # compute the cost and heuristic for the successor node
            successor_total_cost = current_node[2] + successor_cost
            successor_heuristic = heuristic_2(problem, robot_id, successor_state)
            successor_node = [successor_state, current_node[1] + [[successor_state[0], successor_state[1]]], successor_total_cost, successor_heuristic + successor_total_cost]
               
            # check if the successor node is already in the explored set
            if successor_state in explored:
                continue
            
            #Check if duplicate node exists in fringe
            flag_do_not_append = False
            for node in fringe:        
                if node[0] == successor_state:                  
                    #Check if existing duplicate is actually shorter path than the new node            
                    if node[2] <= successor_node[2]:
                        #In this case do not add the new node to fringe 
                        flag_do_not_append = True
                        #No need to check further in existing fringe
                        break
          
            if flag_do_not_append:
                #In this case do not add the new node 
                continue
          
            #If none of the above then add successor to fringe 
            fringe.append(successor_node)
                
    # if the fringe is empty and the goal state was not found, return failure
    return ([], 'failure')

