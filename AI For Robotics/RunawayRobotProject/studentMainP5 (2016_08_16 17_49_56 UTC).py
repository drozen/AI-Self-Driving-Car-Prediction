# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

# some top level parameters
#

max_steering_angle = pi / 4.0 # You do not need to use this value, but keep in mind the limitations of a real car.
bearing_noise = 0.1 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 5.0 # Noise parameter: should be included in move function.

tolerance_xy = 15.0 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.


# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"


# --------
#
# extract position from a particle set
# 

def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]

# --------
#
# The following code generates the measurements vector
# You can use it to develop your solution.
# 

# gives us a sequence of measurements and a robot position using a a robot simulation

def generate_ground_truth(motions):

    myrobot = robot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

    Z = []
    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    #print 'Robot:    ', myrobot
    return [myrobot, Z]

# --------
#
# The following code prints the measurements associated
# with generate_ground_truth
#

def print_measurements(Z):

    T = len(Z)

    print 'measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
        (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3]))
    for t in range(1,T-1):
        print '                [%.8s, %.8s, %.8s, %.8s],' % \
            (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3]))
    print '                [%.8s, %.8s, %.8s, %.8s]]' % \
        (str(Z[T-1][0]), str(Z[T-1][1]), str(Z[T-1][2]), str(Z[T-1][3]))

# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#

def check_output(final_robot, estimated_position):

    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy \
              and error_orientation < tolerance_orientation
    return correct



def particle_filter(motions, measurements, N=500): # I know it's tempting, but don't change N!
    # --------
    #
    # Make particles
    # 

    p = []
    for i in range(N):
        r = robot()
        r.set_noise(bearing_noise, steering_noise, distance_noise)
        p.append(r)

    # --------
    #
    # Update particles
    #     

    for t in range(len(motions)):
    
        # motion update (prediction)
        p2 = []
        for i in range(N):
            p2.append(p[i].move(motions[t]))
        p = p2

        # measurement update - measurement probability function which is part of implementation
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(measurements[t]))

        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3
    
    return get_position(p)


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
        xy_estimate = (OTHER[0][0][0], OTHER[0][0][1])

        heading_to_target = get_heading(hunter_position, xy_estimate)
        heading_difference = heading_to_target - hunter_heading
        turning =  heading_difference # turn towards the target
        distance = max_distance      
        
    elif len(OTHER[0]) == 1:
        
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
 
        # keep track of other measurement 
 
        #calculate the step size, where OTHER is the previous measurement and measurement is the current measurement

        stepSize = distance_between(OTHER[0][0], measurements[0])
        x2 = measurements[0][0]
        x1 = OTHER[0][0][0]
         
        y2 = measurements[0][1] 
        y1 = OTHER[0][0][1]
          
        xy_estimate = (x2+x1, y2+y1)
        
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables

        heading_to_target = get_heading(hunter_position, xy_estimate)
        heading_difference = heading_to_target - hunter_heading
        turning =  heading_difference # turn towards the target
        distance = max_distance # full speed ahead!
        return turning, distance, OTHER

    else: # not the first time, update my history
        
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        
        
        x3 = measurements[0][0] 
        x2 = OTHER[0][(len(OTHER[0])-1)][0]
        x1 = OTHER[0][len(OTHER[0])-2][0]
        
        y3 = measurements[0][1] 
        y2 = OTHER[0][(len(OTHER[0])-1)][1]
        y1 = OTHER[0][(len(OTHER[0])-2)][1]
        
        stepSize = distance_between((x2,y2), measurements[0])
                
        # find centre of circle 
        
        # first find slopes
        
        m1 = (y2-y1) / (x2- x1)
        
        m2 = (y3-y2) / (x3- x2)
        
        # calculate centre x coordinate
        xc = ( (m1*m2*(y3-y1)) + m1*(x2+x3) -m2*(x1+x2) ) / (2*(m1-m2)) 
        # y coordinate
        yc = -1*(1./m1)*( xc-((x1+x2)/.2) ) + (y1+y2)/.2
        
        #radius
        rc = distance_between((x3,y3),(xc,yc))

                # calculate deltas for angle
        dx = x3 - x2
        dy = y3 - y2
        
        #initial turning angle
        
        angle1 = atan2((y2-y1),(x2-x1))
        angle2 = atan2(dy,dx)    
        
        
        
        motions = [[angle2, stepSize]]
        
        measurements = [[x3, y3]]
        
        print 'motions', motions
        print 'measurements', measurements

        (x3p, y3p, angle2p) = particle_filter(motions, measurements, 50)
    
        
        #angle change
        dangle = angle2p - angle1
        
        newAngle = (angle2p + dangle) % (2*pi)
        
        ################### TODO: vary the future position####################
      
        newx = x3p + (stepSize * cos(newAngle))
        newy = y3p + (stepSize * sin(newAngle))
        
        xy_estimate = (newx, newy)
        
        print  'x3p', x3p, 'y3p', y3p, 'x3', x3,',', y3, ',', xc, ',', yc , ',', rc
        
        ## Move n steps ahead
        
        n = 1
        
        # in order to move less than n steps ahead, multiply by mod
        mod = .8
        
        for i in range(n):
            newAngle += dangle * mod
            newAngle %= 2*pi
            newx += (stepSize * cos(newAngle) *mod)
            newy += (stepSize * sin(newAngle) *mod)
            
        

        # xy_estimate = (newx, newy)
        
        print  "xy_est", xy_estimate
        
            
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
        heading_to_target = get_heading(hunter_position, xy_estimate)
        heading_difference = heading_to_target - hunter_heading
        turning =  heading_difference # turn towards the target
        distance = max_distance # full speed ahead!
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

## VISUAL DEMO GRADING

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

# ## NON-VISUAL ORIGINAL DEMO-GRADING
# 
# def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
#     """Returns True if your next_move_fcn successfully guides the hunter_bot
#     to the target_bot. This function is here to help you understand how we 
#     will grade your submission."""
#     max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
#     separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
#     caught = False
#     ctr = 0
# 
#     # We will use your next_move_fcn until we catch the target or time expires.
#     while not caught and ctr < 1000:
# 
#         # Check to see if the hunter has caught the target.
#         hunter_position = (hunter_bot.x, hunter_bot.y)
#         target_position = (target_bot.x, target_bot.y)
#         separation = distance_between(hunter_position, target_position)
#         if separation < separation_tolerance:
#             print "You got it right! It took you ", ctr, " steps to catch the target."
#             caught = True
# 
#         # The target broadcasts its noisy measurement
#         target_measurement = target_bot.sense()
# 
#         # This is where YOUR function will be called.
#         turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
#         
#         # Don't try to move faster than allowed!
#         if distance > max_distance:
#             distance = max_distance
# 
#         # We move the hunter according to your instructions
#         hunter_bot.move(turning, distance)
# 
#         # The target continues its (nearly) circular motion.
#         target_bot.move_in_circle()
# 
#         ctr += 1            
#         if ctr >= 1000:
#             print "It took too many steps to catch the target."
#     return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER
 
target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)
 
hunter = robot(-10.0, -10.0, 0.0)
 
print demo_grading(hunter, target, next_move)

# ## --------
# ## TEST CASES:
# ## 
# ##1) Calling the particle_filter function with the following
# ##    motions and measurements should return a [x,y,orientation]
# ##    vector near [x=93.476 y=75.186 orient=5.2664], that is, the
# ##    robot's true location.
# ##
# motions = [[2. * pi / 10, 20.] for row in range(8)]
#  ## These are the bearings to those 4 different landmarks
# measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
#                 [3.510067, 2.916300, 2.146394, 1.598332],
#                 [2.972469, 2.407489, 1.588474, 1.611094],
#                 [1.906178, 1.193329, 0.619356, 0.807930],
#                 [1.352825, 0.662233, 0.144927, 0.799090],
#                 [0.856150, 0.214590, 5.651497, 1.062401],
#                 [0.194460, 5.660382, 4.761072, 2.471682],
#                 [5.717342, 4.736780, 3.909599, 2.342536]]
#  
# print particle_filter(motions, measurements)
#    
# #  2) You can generate your own test cases by generating
# #     measurements using the generate_ground_truth function.
# #     It will print the robot's last location when calling it.
# #  
#   
# number_of_iterations = 6
# motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
#   
#   # 
#   #     "Generate<u>ground<u>truth" gives us a sequence of measurements and 
#   # a robot position that we can split as follows
#   # , using a robot simulation.
#     
# 
# estimated_position = particle_filter(motions, measurements)  # particle filter estimated position
# print_measurements(measurements)
# print 'Particle filter: ', estimated_position





