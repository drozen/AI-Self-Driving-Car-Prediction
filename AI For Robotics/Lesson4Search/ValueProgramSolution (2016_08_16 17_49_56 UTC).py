# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

import pprint

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    
    # I now update the value function a number times--I don't know how often--
    # but as long as change something, I update it.
    # Therefore, I introduced the variable "change," which I set to True in the beginning.
    change = True
    
    while change:
        change = False
        
        # go through all the grid cells 
        
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                
                # if it's a goal cell 
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        # I changed the value to 0, therefore I set the flag back to True
                        change = True
                        
                # If it's not a goal cell, here's my full update function
                elif grid[x][y] == 0:
                    # I go through all the actions (1:22)
                    for a in range(len(delta)):
                        # project a potential next state upon executing an action
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]
                        
                        # test whether x2, y2 are legitimate states
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            
                            # then propogate back the value 
                            v2 = value[x2][y2] + cost
                            
                            # if v2 is better (Smaller) than the current value, 
                            if v2 < value[x][y]:
                                change =  True
                                # then I assign this new value to my original grid cell x and y, 
                                # plus the cost step. 
                                value[x][y] = v2
                            
                    
                        
    
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    for i in range(len(value)):
        print value[i]
    
    return value 



compute_value(grid,goal,cost)