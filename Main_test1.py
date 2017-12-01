__author__ = 'agmadi'

import sys
from math import *
from rangeFinder import RangeFinder
from matrix import matrix

def vectorize(measurementVector):
    x, y, heading, velocity, yawRate = measurementVector
    return matrix([
            [x],
            [y],
            [velocity],
            [yawRate]
        ])

def matricize(measurementVector):
    x, y, heading, velocity, yawRate = measurementVector
    return matrix([
            [x],
            [y],
            [heading],
            [velocity],
            [yawRate]
        ])

def decomposeVector(vctr):
    return [ x[0] for x in vctr.value ]

def predict_EKF(measurementVector, dt):
    x, y, heading, velocity, yawRate = measurementVector
    if yawRate < 0.0001:
        #Driving straight - avoid division by zero (yr < 0.0001)
        x = x + velocity * dt * cos(heading)
        y = y + velocity*dt * sin(heading)
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

    # the 3 lines below are new and do different calcs as an experiment
    x = x + velocity * cos(heading + yawRate)
    y = y + velocity * sin(heading + yawRate)
    heading = heading + yawRate

    return [ x, y, heading, velocity, yawRate ]

def createPredictionJacobian(measurementVector, dt):
    # I'm overriding the calcs with different ones as an experiment

    x, y, heading, velocity, yawRate = measurementVector
    a_1_3 = float((velocity/yawRate) * (cos(yawRate*dt+heading) - cos(heading)))
    a_1_3 = float(-velocity*sin(heading+yawRate))
    a_1_4 = float((1.0/yawRate) * (sin(yawRate*dt+heading) - sin(heading)))
    a_1_4 = float(cos(heading+yawRate))
    a_1_5 = float((dt*velocity/yawRate)*cos(yawRate*dt+heading) - (velocity/yawRate**2)*(sin(yawRate*dt+heading) - sin(heading)))
    a_1_5 = float(-velocity*sin(heading+yawRate))
    a_2_3 = float((velocity/yawRate) * (sin(yawRate*dt+heading) - sin(heading)))
    a_2_3 = float(velocity*cos(heading+yawRate))
    a_2_4 = float((1.0/yawRate) * (-cos(yawRate*dt+heading) + cos(heading)))
    a_2_4 = float(sin(heading+yawRate))
    a_2_5 = float((dt*velocity/yawRate)*sin(yawRate*dt+heading) - (velocity/yawRate**2)*(-cos(yawRate*dt+heading) + cos(heading)))
    a_2_5 = float(velocity*cos(heading+yawRate))
    return matrix( [
        [1.0, 0.0, a_1_3, a_1_4, a_1_5],
        [0.0, 1.0, a_2_3, a_2_4, a_2_5],
        [0.0, 0.0, 1 + yawRate, 0.0, 1 + heading],
        #[0.0, 0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0],
    ])

def createMeasurementJacobian(measurementVector):
    # I added an extra row in each matrix as an experiment (3rd row)
    x, y, heading, velocity, yawRate = measurementVector
    if x == -1 and y == -1:
        # bad data so don't use it
        return matrix([
            [0.,0.,0.,0.,0.],
            [0.,0.,0.,0.,0.],
            [0.,0.,1.,0.,0.],
            [0.,0.,0.,1.,0.],
            [0.,0.,0.,0.,1.]
        ])
    # data is fine so return proper matrix
    return matrix([
            [1.,0.,0.,0.,0.],
            [0.,1.,0.,0.,0.],
            [0.,0.,1.,0.,0.],
            [0.,0.,0.,1.,0.],
            [0.,0.,0.,0.,1.]
        ])

def extended_kalman_filter(extendedData, x, P, Q, R, I, dt):
    # Doing some experimentation so I made some changes but left the old suff.
    # changes just override old calculations.

    for n in range(len(extendedData)):

        # prediction for EKF
        x = predict_EKF(x, dt)
        JacobianF = createPredictionJacobian(x, dt)
        P = JacobianF * P * JacobianF.transpose() + Q

        # measurement update for EKF
        JacobianH = createMeasurementJacobian(extendedData[n])
        hOFx = vectorize(x) # measurement function (h(x)). See comments above
        hOFx = matricize(x) # measurement function (h(x)). See comments above
        Z = vectorize(extendedData[n])
        Z = matricize(extendedData[n])
        y = Z - hOFx
        S = JacobianH * P * JacobianH.transpose() + R
        K = P * JacobianH.transpose() * S.inverse()
        x = decomposeVector(matricize(x) + (K * y))
        P = I - ((K * JacobianH) * P)

    print 'x= '
    matricize(x).show()
    print 'P= '
    P.show()

def readDataFile( fileName ):
    f = open( fileName )
    allLines = f.read().replace('\n','')
    return eval( allLines )

def main():
    #fileName = sys.argv[1]
    # for debugging just hardcode name

    dataMaxX = dataMaxY = 0
    fileName = "training_video1-centroid_data"
    rawData = readDataFile( fileName )
    r = RangeFinder(rawData)
    r.updateAll()
    extendedData = r.getNewData()
    dataMaxX, dataMaxY = r.getRange()

    initial_data = extendedData[0]

    # dt = 1.0/24.0   #0.04166666667
    dt = 1.0

    # For EKF, we will need [ x, y, heading (h), velocity, yaw rate ]
    # x = matrix( [[initial_data[0]],
    #             [initial_data[1]],
    #             [initial_data[2]],
    #             [initial_data[3]],
    #             [initial_data[4]]]) # initial state (location and velocity)
    x = [
        initial_data[0],
        initial_data[1],
        initial_data[2],
        initial_data[3],
        initial_data[4]
    ] # initial state (location, heading, velocity, and yaw rate)

    # For EKF, I don't think we need this matrix
    # u = matrix([[0.], [0.], [0.], [0.]]) # external motion

    # For EKF, this could start as a 5x5 matrix for 1000.0 on the diagonal, or perhaps the one here.
    P =  matrix([[1000., 0., 0., 0., 0.],
                 [0., 1000., 0., 0., 0.],
                 [0., 0., 1000., 0., 0.],
                 [0., 0., 0., 1000., 0.],
                 [0., 0., 0., 0., 1000.]]) # initial uncertainty

    # For EKF, we should use the Movement/Prediction jacobian matrix from the comments above
    # We'll also need Q, the process noice covariance matrix, which we'll need to figure out. See comments above.
    # F =  matrix([[1., 0., dt, 0.],
    #              [0., 1., 0., dt],
    #              [0., 0., 1., 0.],
    #              [0., 0., 0., 1.]]) # next state function
    Q =  matrix([[1., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0.],
                 [0., 0., 1., 0., 0.],
                 [0., 0., 0., 1., 0.],
                 [0., 0., 0., 0., 1.]])

    # For EKF, this needs to be the Measurement jacobian from the comments above
    # H =  matrix([[1., 0., 0., 0.],
    #              [0., 1., 0., 0.]]) # measurement function

    # For EKF, this needs to be just a 4x4 or 5x5 identity matrix (leaning towards 4x4)
    # R =  matrix([[0.1, 0.],
    #              [0., 0.1]]) # measurement uncertainty 2x2 (0.1)
    R = matrix([[0.1, 0., 0., 0.],
             [0., 0.1, 0., 0.],
             [0., 0., 0.1, 0.],
             [0., 0., 0., 0.1]])
    R = matrix([[1., 0., 0., 0., 0.],
             [0., 1., 0., 0., 0.],
             [0., 0., 1., 0., 0.],
             [0., 0., 0., 1., 0.],
             [0., 0., 0., 0., 1.]])

    I =  matrix([[1., 0., 0., 0., 0.],
                 [0., 1., 0., 0., 0.],
                 [0., 0., 1., 0., 0.],
                 [0., 0., 0., 1., 0.],
                 [0., 0., 0., 0., 1.]]) # identity matrix

    extended_kalman_filter(extendedData, x, P, Q, R, I, dt)

if __name__ == '__main__':
    main()

