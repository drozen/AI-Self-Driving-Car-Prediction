# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    
    if not OTHER: # this is the first measurement
        OTHER = measurement
        xy_estimate = OTHER


    
    elif len(OTHER) == 2:
 
        # keep track of other measurement 
 
 
        #calculate the step size, where OTHER is the previous measurement and measurement is the current measurement
 
        stepSize = distance_between(OTHER[0:2], measurement)
        x2 = measurement[0] 
        x1 = OTHER[len(OTHER)-2]
         
        y2 = measurement[1] 
        y1 = OTHER[len(OTHER)-3]
#          
        xy_estimate = (x2+x1, y2+y1)
#         print xy_estimate, OTHER

        OTHER += measurement


        
    else:
        
        # keep track of other measurement 

        x3 = measurement[0] 
        x2 = OTHER[(len(OTHER)-2)]
        x1 = OTHER[len(OTHER)-4]
        
        y3 = measurement[1] 
        y2 = OTHER[(len(OTHER)-1)]
        y1 = OTHER[len(OTHER)-3]
        
        stepSize = distance_between((x2,y2), measurement)

        
        # calculate deltas for angle
        dx = x3 - x2
        dy = y3 - y2
        
        #initial turning angle
        
        angle1 = atan2((y2-y1),(x2-x1))
        angle2 = atan2(dy,dx)        
        
        #angle change
        dangle = angle2 - angle1
        
        newAngle = (angle2 + dangle) % (2*pi)
      
        newx = x3 + stepSize * cos(newAngle)
        newy = y3 + stepSize * sin(newAngle)
#                 # calculate the chords

        xy_estimate = (newx, newy)


#         # chord between p3 and p2
#         d1 = distance_between((x2,y2), (x3,y3))
#         
#         # chord between p3 and p1
#         d2 = distance_between((x1,y1), (x3,y3))
#         
#         newx = x3 + (d2/d1) * ( cos(angle2)*(x3 - x2) - sin(angle2)*(y3 - y2) )
#         newy = y3 + (d2/d1) * ( sin(angle2)*(x3 - x2) + cos(angle2)*(y3 - y2) )
#          
#         newx = x3 + d2/d1 * ( cos(angle)(x3 - x2) - sin(y3 - y2) )
#         newy = y3 + d2/d1 * ( sin(x3 - x2) + cos(y3 - y2) )

#         newx = (x3+x2)
#         newy = (y3+y2)
        
        
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
   
        OTHER += measurement

 
    return xy_estimate, OTHER
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    xy_estimate = (3.2, 9.1)
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(naive_next_pos, test_target)




