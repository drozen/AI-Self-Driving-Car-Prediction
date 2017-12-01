#!/usr/bin/env python
__author__ = 'agmadi'

import sys, time, turtle
from numpy import matrix, eye, diag, abs, sin, cos, pi, linalg
from rangeFinder import RangeFinder

PREDICT_FRAMES = 60

def vectorize(measurementVector):
    x, y, heading, velocity, yawRate = measurementVector
    return matrix([x, y, velocity, yawRate]).T

def predict_EKF(stateVector, dt):
    x, y, heading, velocity, yawRate = stateVector
    if abs(yawRate) < 0.0001:
        #Driving straight - avoid division by zero (yr < 0.0001)
        x = x + velocity * dt * cos(heading)
        y = y + velocity * dt * sin(heading)
        heading = heading
        velocity = velocity
        yawRate = 0.0000001 # avoid numerical issues in Jacobians
    else:
        # Driving not so straight
        x = x + (velocity/yawRate) * (sin(yawRate*dt+heading) - sin(heading))
        y = y + (velocity/yawRate) * (-cos(yawRate*dt+heading) + cos(heading))
        heading = (heading + yawRate*dt + pi) % (2.0*pi) - pi
        velocity = velocity
        yawRate = yawRate

    return matrix([ float(x), float(y), float(heading), float(velocity), float(yawRate) ]).T

def createPredictionJacobian(stateVector, dt):
    x, y, heading, velocity, yawRate = stateVector
    a_1_3 = float((velocity/yawRate) * (cos(yawRate*dt+heading) - cos(heading)))
    a_1_4 = float((1.0/yawRate) * (sin(yawRate*dt+heading) - sin(heading)))
    a_1_5 = float((dt*velocity/yawRate)*cos(yawRate*dt+heading) - (velocity/yawRate**2)*(sin(yawRate*dt+heading) - sin(heading)))
    a_2_3 = float((velocity/yawRate) * (sin(yawRate*dt+heading) - sin(heading)))
    a_2_4 = float((1.0/yawRate) * (-cos(yawRate*dt+heading) + cos(heading)))
    a_2_5 = float((dt*velocity/yawRate)*sin(yawRate*dt+heading) - (velocity/yawRate**2)*(-cos(yawRate*dt+heading) + cos(heading)))

    return matrix( [
        [1.0, 0.0, a_1_3, a_1_4, a_1_5],
        [0.0, 1.0, a_2_3, a_2_4, a_2_5],
        [0.0, 0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
    ])

def createMeasurementJacobian(measurementVector):
    x, y, heading, velocity, yawRate = measurementVector
    if x == -1 and y == -1:
        # bad data so don't use it
        return matrix([
            [0.,0.,0.,0.,0.],
            [0.,0.,0.,0.,0.],
            [0.,0.,0.,1.,0.],
            [0.,0.,0.,0.,1.]
        ])
    # data is fine so return proper matrix
    return matrix([
            [1.,0.,0.,0.,0.],
            [0.,1.,0.,0.,0.],
            [0.,0.,0.,1.,0.],
            [0.,0.,0.,0.,1.]
        ])

def extended_kalman_filter(extendedData, x, P, Q, R, I, dt):
    # Doing some experimentation so I made some changes but left the old suff.
    # changes just override old calculations.

    for n in range(len(extendedData)):

        # measurement update for EKF
        JacobianH = createMeasurementJacobian(extendedData[n])
        hOFx = matrix([
            [float(x[0])],
            [float(x[1])],
            [float(x[3])],
            [float(x[4])]
        ])
        Z = vectorize(extendedData[n])
        y = Z - hOFx
        S = JacobianH * P * JacobianH.T + R
        K = (P * JacobianH.T) * linalg.inv(S)
        x = x + (K * y)
        P = I - ((K * JacobianH) * P)

        # prediction for EKF
        x = predict_EKF(x, dt)
        JacobianF = createPredictionJacobian(x, dt)
        P = JacobianF * P * JacobianF.T + Q

    # print 'x= ', x
    # print 'P= ', P

    return x, P

def readDataFile( fileName ):
    f = open( fileName )
    allLines = f.read().replace('\n','')
    return eval( allLines )

def main():
    #fileName = sys.argv[1]
    # for debugging just hardcode name

    dataMinX = dataMinY = dataMaxX = dataMaxY = 0
    fileName = "training_video1-centroid_data"
    # fileName = "training_data_1"
    # fileName = "training_data_2"
    #fileName = "training_data_3"
    #fileName = "training_data_4"
    rawData = readDataFile( fileName )
    r = RangeFinder(rawData)
    r.updateAll()
    extendedData = r.getNewData()
    dataMinX, dataMinY, dataMaxX, dataMaxY = r.getRange()

    initial_data = extendedData[0]

    #dt = 1.0/24.0   #0.04166666667
    dt = 1.0

    # For EKF, we will need [ x, y, heading (h), velocity, yaw rate ]
    x = matrix([
        [initial_data[0],
         initial_data[1],
         initial_data[2],
         initial_data[3],
         initial_data[4]]
    ]).T # initial state (location, heading, velocity, and yaw rate)

    # For EKF, I don't think we need this matrix
    # u = matrix([[0.], [0.], [0.], [0.]]) # external motion

    # For EKF, this could start as a 5x5 matrix for 1000.0 on the diagonal, or perhaps the one here.
    P =  diag([1000.0, 1000.0, 1000.0, 1000.0, 1000.0]) # initial uncertainty

    # For EKF, we should use the Movement/Prediction jacobian matrix from the comments above
    # We'll also need Q, the process noice covariance matrix, which we'll need to figure out. See comments above.
    # F =  matrix([[1., 0., dt, 0.],
    #              [0., 1., 0., dt],
    #              [0., 0., 1., 0.],
    #              [0., 0., 0., 1.]]) # next state function
    Q = diag([1., 1., 1., 1., 1.])

    # For EKF, this needs to be the Measurement jacobian from the comments above
    # H =  matrix([[1., 0., 0., 0.],
    #              [0., 1., 0., 0.]]) # measurement function

    # For EKF, this needs to be just a 4x4 or 5x5 identity matrix (leaning towards 4x4)
    # R =  matrix([[0.1, 0.],
    #              [0., 0.1]]) # measurement uncertainty 2x2 (0.1)
    R = matrix([
        [0.1, 0., 0., 0.],
        [0., 0.1, 0., 0.],
        [0., 0., 0.1, 0.],
        [0., 0., 0., 0.1],
    ])
    # R = matrix([[1., 0., 0., 0., 0.],
    #          [0., 1., 0., 0., 0.],
    #          [0., 0., 1., 0., 0.],
    #          [0., 0., 0., 1., 0.],
    #          [0., 0., 0., 0., 1.]])

    I = eye( 5 ) # identity matrix

    stateVector, PVector = extended_kalman_filter(extendedData, x, P, Q, R, I, dt)

    print 'X1=', stateVector

    stateVector[3] = r.getMediaVelocity()
    print 'Median Velocity is', r.getMediaVelocity()
    print 'min', dataMinX, dataMinY
    print 'max', dataMaxX, dataMaxY

    # Init visualization window
    window = turtle.Screen()
    window.bgcolor('white')

    # Set offset variables so drawing is more easily viewable
    xOffset = 400
    yOffset = 200

    # Init hexbug object
    hexbug_visual = turtle.Turtle()
    hexbug_visual.shape('arrow')
    hexbug_visual.color('blue')
    hexbug_visual.resizemode('user')
    hexbug_visual.shapesize(.25, .25, .25)
    hexbug_visual.penup()

    # Draw box bounds based on min and max data
    bounds = turtle.Turtle()
    bounds.shape('circle')
    bounds.color('red')
    bounds.resizemode('user')
    bounds.shapesize(.05, .05, .05)
    bounds.penup()
    boundsMinX = dataMinX - xOffset
    boundsMaxX = dataMaxX - xOffset
    boundsMinY = dataMinY - yOffset
    boundsMaxY = dataMaxY - yOffset
    bounds.goto(boundsMinX, boundsMinY)
    bounds.pendown()
    bounds.goto(boundsMaxX, boundsMinY)
    bounds.goto(boundsMaxX, boundsMaxY)
    bounds.goto(boundsMinX, boundsMaxY)
    bounds.goto(boundsMinX, boundsMinY)

    # for 60 times call predictor
    for i in xrange(PREDICT_FRAMES):
        # fix yaw rate since it's so very noisy
        YAW_LIMIT = 0.05
        if stateVector[4] < -YAW_LIMIT:
            stateVector[4] = -YAW_LIMIT
        elif stateVector[4] > YAW_LIMIT:
            stateVector[4] = YAW_LIMIT

        # call the predictor function to get the next prediction
        stateVector = predict_EKF(stateVector, dt)
        x, y, heading, velocity, yawRate = stateVector
        print x, y

        hexbug_visual.setheading(heading[0,0]*180/pi)
        hexbug_visual.goto(x[0,0]- xOffset, y[0,0] - yOffset)
        hexbug_visual.stamp()

        time.sleep(.1)

        # make sure we don't hit a wall before calling next step
        # check x and y against walls ( dataMinX, dataMinY, dataMaxX, dataMaxY )
        # if x <= dataMinX and y <= dataMinY:
        #     pass # bounce it off top left corner
        # elif x <= dataMinX and y >= dataMaxY:
        #     pass # bounce it off bottom left corner
        # elif x >= dataMaxX and y <= dataMinY:
        #     pass # bounce it off top right corner
        # elif x >= dataMaxX and y >= dataMaxY:
        #     pass # bounce it off bottom right corner
        if x <= dataMinX:
            stateVector[2] = r.angle_trunc(pi-stateVector[2]) # bounce it off left wall
            stateVector[0] = dataMinX
            print 'bounce1'
        elif x >= dataMaxX:
            stateVector[2] = r.angle_trunc(pi-stateVector[2]) # bounce it off right wall
            stateVector[0] = dataMaxX
            print 'bounce2'
        elif y <= dataMinY:
            stateVector[2] = r.angle_trunc(-stateVector[2]) # bounce it off top wall
            stateVector[1] = dataMinY
            print 'bounce3'
        elif y >= dataMaxY:
            stateVector[2] = r.angle_trunc(-stateVector[2]) # bounce it off bottom wall
            stateVector[1] = dataMaxY
            print 'bounce4'

    print 'X2=', stateVector
    print 'Press ENTER to end'
    sys.stdin.readline()


if __name__ == '__main__':
    main()
