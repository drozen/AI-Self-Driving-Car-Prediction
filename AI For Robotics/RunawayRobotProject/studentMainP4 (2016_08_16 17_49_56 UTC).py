# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

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
        
        #angle change
        dangle = angle2 - angle1
        
        newAngle = (angle2 + dangle) % (2*pi)
        
        ################### TODO: vary the future position####################
      
        newx = x3 + (stepSize * cos(newAngle))
        newy = y3 + (stepSize * sin(newAngle))
        
        xy_estimate = (newx, newy)
        
        print  x3,',', y3, ',', xc, ',', yc , ',', rc
        
        ## Move n steps ahead
        
        n = 1
        
        # in order to move less than n steps ahead, multiply by mod
        mod = .8
        
        for i in range(n):
            newAngle += dangle * mod
            newAngle %= 2*pi
            newx += (stepSize * cos(newAngle) *mod)
            newy += (stepSize * sin(newAngle) *mod)

        xy_estimate = (newx, newy)
        
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

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

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

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



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

# target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
# measurement_noise = .05*target.distance
# target.set_noise(0.0, 0.0, measurement_noise)

# hunter = robot(-10.0, -10.0, 0.0)

# print demo_grading(hunter, target, naive_next_move)





