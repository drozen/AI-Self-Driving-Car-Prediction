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

sensor_wrong = 1.0 - sensor_right

p_move = 0.8

p_stay = 1.0 - p_move


def show(p):
    for i in range(len(p)):
        print p[i]

#Do not delete this comment!
#Do not use any import statements.
#Adding or changing any code above may
#cause the assignment to be graded incorrectly.

#Enter your code here:


if len(measurements) != len(motions):
    raise ValueError, "error in size of measurement/motion vector"

pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    
p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    

# 2. define sense function

def sense(p, colors, measurement):
    
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    
    s = 0.0
    
    for i in range(len(p)):
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j])
            aux[i][j] = p[i][j]*(hit*sensor_right + (1 - hit) * (1- sensor_right)))
            s += aux[i][j]
    
    for i in range(len(aux)):
        for j in range(len(p[i])):
            aux[i][j] /= s
            
    return aux

# 3. define move function

def move(p, motion):
    # Introduce auxilary variable s
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]

    
    for i in range(len(p)):
        for j in range(len(p[i])):
            aux[i][j] = (p_move * p[(i - motion[0])%len(p)][(j - motion[1]) % len(p[0]) + ((1 - p_move) * p[i][j]))
            
    return q
        
    


# 4. execute move then motion according to input

for k in range(len(measurements)):
    p = move(p, motions[k])
    p = sense(p, colors, measurements[k])
    
#Your probability array must be printed 
#with the following code.

show(p)
