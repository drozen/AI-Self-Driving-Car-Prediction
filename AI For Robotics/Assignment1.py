#colors = [['red', 'green', 'green',   'red', 'red'],
#          ['red',   'red', 'green',   'red', 'red'],
#          ['red',   'red', 'green', 'green', 'red'],
#          ['red',   'red',   'red',   'red', 'red']]
#
#measurements = ['green', 'green', 'green' ,'green', 'green']

colors = [['green', 'green','green'],
        ['green', 'red', 'red'],
        ['green', 'green', 'green']]
        
measurements = ['red', 'red']

motions = [[0,0], [0,1]]

#
#
#p = [[1./20, 1./20, 1./20, 1./20, 1./20],
#     [1./20, 1./20, 1./20, 1./20, 1./20],
#     [1./20, 1./20, 1./20, 1./20, 1./20],
#     [1./20, 1./20, 1./20, 1./20, 1./20]]

#motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

sensor_right = 1.0

p_move = 0.5

def show(p):
    for i in range(len(p)):
        print p[i]

#Do not delete this comment!
#Do not use any import statements.
#Adding or changing any code above may
#cause the assignment to be graded incorrectly.

#Enter your code here:

# get coordinates of colors

x = len(colors)
y = len(colors[0])
n = x * y


# initiliazation of the probability array 
p = []
for i in range(x):
    p.append([])
    for j in range(y):
         p[i].append(1/float(n))


def sense(p, Z):
    q = []
    for i in range(x):
        q.append([])
        for j in range(y):
            hit = (Z == colors[i][j])
            q[i].append( p[i][j] * (hit * sensor_right + (1-hit) * (1-sensor_right)))
   
    
    # normalize
    
    # first compute the total sum
    
    s = 0
    for i in range(x):
        for j in range(y):
            s += q[i][j]
            
    # now normalize with this s 
    
    for i in range(x):
        for j in range(y):
            q[i][j] = q[i][j] / s
            
    return q

def move(p, U):
    q = []
    for i in range(x):
        q.append([])
        for j in range(y):
            q[i].append(((p[(i-U[0])%x][(j-U[1])%y]*p_move))+p[i][j]*(1-p_move))   ### add in the probability of not moving by accident
    return q

for i in range(len(measurements)):
    p = move(p, (motions[i]))
    p = sense(p, (measurements[i]))
    
#Your probability array must be printed 
#with the following code.

show(p)
