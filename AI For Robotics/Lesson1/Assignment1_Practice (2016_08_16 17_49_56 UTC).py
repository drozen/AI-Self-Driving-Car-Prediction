colors = [['red', 'green', 'green',   'red', 'red'],
          ['red',   'red', 'green',   'red', 'red'],
          ['red',   'red', 'green', 'green', 'red'],
          ['red',   'red',   'red',   'red', 'red']]

measurements = ['green', 'green', 'green' ,'green', 'green']

#colors = [['green', 'green','green'],
#        ['green', 'red', 'red'],
#        ['green', 'green', 'green']]
#        
#measurements = ['red', 'red']

#motions = [[0,0], [0,1]]



motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

sensor_right = 0.7

p_move = 0.8

def show(p):
    for i in range(len(p)):
        print p[i]

#Do not delete this comment!
#Do not use any import statements.
#Adding or changing any code above may
#cause the assignment to be graded incorrectly.

#Enter your code here:


# 1. initialize probability distribution 

x = len(colors)
y = len(colors[0])
n = x * y

p = []

for i in range(x):
    p.append([])
    for j in range(y):
        p[i].append((1./n))


# 2. define sense function

def sense(p, Z):
    q = []
    
    for i in range(x):
        q.append([])
        for j in range(y):
            hit = (Z == colors[i][j])
            q[i].append(p[i][j]*(hit*sensor_right + (1 - hit) * (1- sensor_right)))
            
        
    sum = 0
    
    for i in range(x):
        for j in range(y):
            sum += q[i][j]
    
    for i in range(x):
        for j in range(y):
            q[i][j] /= sum
            
    return q

# 3. define move function

def move(p, U):
    # Introduce auxilary variable s
    q = []
    
    for i in range(x):
        q.append([])
        for j in range(y):
            q[i].append(((p_move * p[(i - U[0])%x][(j - U[1]) % y])) + ((1 - p_move) * p[i][j]))
            
    return q
        
    


# 4. execute move then motion according to input

for k in range(len(measurements)):
    p = move(p, motions[k])
    p = sense(p, measurements[k])
    
#Your probability array must be printed 
#with the following code.

show(p)
