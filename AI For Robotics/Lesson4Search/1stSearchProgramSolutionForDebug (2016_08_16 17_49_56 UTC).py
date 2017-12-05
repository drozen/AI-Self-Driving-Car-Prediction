# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, x, y]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]  # initial square
goal = [len(grid)-1, len(grid[0])-1]   # end square
cost = 1

# delta array of the 4 possible movements 
delta = [[-1, 0], # go up   
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    
    # open list elements are of the type: [g, x, y]
    
    # to check cells once expanded I defined an array called "closed" as the same size as my grid
    
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1  # initialize the starting location as checked
    
    # coordinates
    x = init[0]
    y = init[1]
    g = 0  # g-value
    
    # initial open list
    open = [[g, x, y]]
    
    # used 2 flags
    found = False # flag that's set when search is complete
    resign = False # flag set if we can't find a goal position after exploring everything
    
    # print commands for debugging 
    print 'initial open list:'
    for i in range(len(open)):
        print '    ', open [i]
    print '----'
    
    # here's the code:  (go back to 1:35)
    
    # I repeat the following while I haven't found a path to the goal
    # and I haven't proven that the problem is unsolvable.
    
    while found is False and resign is False:
        
        if len(open) == 0:          # check if we still have elements on the open list
            resign = True
            print 'fail'
            #print '##### Search terminated without success'
            
        else:  # if there are still elements
            # remove node from list
            open.sort()
            open.reverse()
            next = open.pop()
            print 'take list item'
            print next
            x = next[1]
            y = next[2]
            g = next[0]
            
            # check if we are done
            
            if x == goal[0] and y == goal[1]:
                found = True
                print next
                print '##### Search successful'
                
            else:   # if not done - here's the interesting case - this is the meat of what I'm programming
                for i in range(len(delta)):  # check out the 4 possibilities
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    # If x2 falls into the grid and y2 falls into the grid 
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 <len(grid[0]):
                        # and [x2, y2] is not yet checked,
                        # which is tested by this field called "closed,"
                        # and the grid cell is navigable--there is no obstacle here.
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2, x2, y2])
                            print 'append list item'
                            print [g2, x2, y2]
                            closed[x2][y2] = 1  # check the coordinate so that I never expand it again
                            
                            
            
                print 'new open list:'
                for i in range(len(open)):
                    print '   ', open[i]
                print '----'
            
    
    
    
    
   #return path    # uncomment this after debugging

search(grid,init,goal,cost)

