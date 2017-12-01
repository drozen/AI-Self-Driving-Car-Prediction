# Modifying code from homework 2 as first test of EKF
#
#   Research
#       http://greg.czerniak.info/guides/kalman1/
#       https://en.wikipedia.org/wiki/Extended_Kalman_filter
#       http://users.isr.ist.utl.pt/~mir/pub/kalman.pdf
#       http://www.goddardconsulting.ca/extended-kalman-filter.html
#       https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter
#       http://www.goddardconsulting.ca/simulink-extended-kalman-filter-tracking.html
#       http://vimeo.com/88057157
#       http://nbviewer.ipython.org/github/balzer82/Kalman/blob/master/Extended-Kalman-Filter-CTRV.ipynb?create=1
#       https://github.com/balzer82/Kalman/blob/master/Extended-Kalman-Filter-CTRV.py
#       http://vimeo.com/88057157
#
#   Prediction:
#         X = g(x, u)
#         P = J P J.tpose + Q
#   Correction:
#         K = P J.tpose ( J P J.tpose + R ).inverse
#         x = x + K (z - h(x))
#         P = (I - K J ) P
#
######
#
#   Measurements must include: [ x, y, heading (h), velocity, yaw rate ]
#   dt - sample rate - 24 fps
#
######
#
#   Movement/Prediction function (update function, g()):
#       x + v / yr ( -sin(h) + sin(Tyr + h) )
#       y + v / yr ( cos(h) - cos(Tyr + h) )
#       Tyr + h
#       v
#       yr
#   Code it up:
#       Driving straight - avoid division by zero (yr < 0.0001)
#       x = x + v*dt * cos(h)
#       y = y + v*dt * sin(h)
#       h = h
#       v = v
#       yr = 0.0000001 # avoid numerical issues in Jacobians
#       Driving not so straight
#       x = x + (v/yr) * (sin(yr*dt+h) - sin(h))
#       y = y + (v/yr) * (-np.cos(yr*dt+h)+ np.cos(h))
#       h = (h + yr*dt + pi) % (2.0*pi) - pi
#       v = v
#       yr = yr
#
#   Movement/Prediction jacobian:
#         a_1_3 = float((v/yr) * (cos(yr*dt+h) - cos(h)))
#         a_1_4 = float((1.0/yr) * (sin(yr*dt+h) - sin(h)))
#         a_1_5 = float((dt*v/yr)*cos(yr*dt+h) - (v/yr**2)*(sin(yr*dt+h) - sin(h)))
#         a_2_3 = float((v/yr) * (sin(yr*dt+h) - sin(h)))
#         a_2_4 = float((1.0/yr) * (-cos(yr*dt+h) + cos(h)))
#         a_2_5 = float((dt*v/yr)*sin(yr*dt+h) - (v/yr**2)*(-cos(yr*dt+h) + cos(h)))
#         1.0, 0.0, a_1_3, a_1_4, a_1_5
#         0.0, 1.0, a_2_3, a_2_4, a_2_5
#         0.0, 0.0, 1.0, 0.0, dt
#         0.0, 0.0, 0.0, 1.0, 0.0
#         0.0, 0.0, 0.0, 0.0, 1.0
#
#   Process noice covariance - Q - sample data  ( used for movement )
#     sGPS     = 0.5*8.8*dt**2  # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
#     sCourse  = 0.1*dt # assume 0.1rad/s as maximum turn rate for the vehicle
#     sVelocity= 8.8*dt # assume 8.8m/s2 as maximum acceleration, forcing the vehicle
#     sYaw     = 1.0*dt # assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle
#     Q = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2, sYaw**2])
#
######
#
#   Measurement function (h()):
#       just keeps the vector unchanged - direct measurement
#       Might only need to be 4x4 - x, y, v, yr = no heading.  not sure about this
#
#   Measurement jacobian:
#       Probably just a 5x5 identity matrix or 5x4 [ [1,0,0,0,0], [0,1,0,0,0], [0,0,0,1,0], [0,0,0,0,1] ]
#       If we have bad data( [-1,-1] ) then [ [0,0,0,0,0], [0,0,0,0,0], [0,0,0,1,0], [0,0,0,0,1] ]
#
#   Measurement covariance - R
#       Probably just a 4x4 or 5x5 identity matrix
#

import sys
from matrix import matrix

def predict_EKF(x):
    # do stuff like in the comments
    return x

def createPredictionJacobian(x):
    # do stuff like in comments
    return x # placeholder

def createMeasurementJacobian(x):
    # do stuff like in comments
    # here we actually will most likely return a 5x5 or 4x4 identity matrix.
    # if we have good data, then place a 1 in the slot for that data, but if we have
    # bad data, as in [-1,-1], then place a 0 in those slots.
    return x # placeholder

def extended_kalman_filter(x, P):
    for n in range(len(measurements)):
        
        # prediction for standard KF
        x = (F * x) + u
        P = F * P * F.transpose()

        # prediction for EKF
        x = predict_EKF(x)
        JacobianF = createPredictionJacobian(x)
        P = JacobianF * P * JacobianF.transpose() + Q

        
        # measurement update for standard KF
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

        # measurement update for EKF
        JacobianH = createMeasurementJacobian(x)
        hOFx = 0 # measurement function (h(x)). See comments above
        Z = 0 # not sure. maybe like above.  needs study.
        y = Z - hOFx # or is it like above: Z.transpose() - hOFx???
        S = JacobianH * P * JacobianH.transpose() + R
        K = P * JacobianH.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * JacobianH)) * P
    
    print 'x= '
    x.show()
    print 'P= '
    P.show()

# For EKF, I don't think we should fix any errors.  It has a way to correct for them.  We just tell the filter
# that no info is available for those points.
measurements = [[584, 189],
                [576, 197],
                [568, 205],
                [560, 215],
                [551, 225],
                [543, 236],
                [537, 247],
                [532, 258],
                [528, 268],
                [525, 278],
                [524, 288],
                [523, 299],
                [522, 311],
                [522, 322],
                [523, 333],
                [525, 345],
                [527, 356],
                [530, 367],
                [533, 379],
                [537, 389],
                [542, 400],
                [547, 416],
                [546, 403],
                [547, 403],
                [551, 416],
                [558, 405],
                [558, 407],
                [584, 414],
                [580, 410],
                [590, 410],
                [599, 414],
                [614, 412],
                [625, 406],
                [631, 408],
                [645, 406],
                [649, 408],     # fixed
                [658, 399],
                [649, 400],
                [658, 401],     # fixed (adding absolute value of difference between previous 2)
                [657, 404],
                [661, 407],
                [670, 420],
                [672, 413],
                [674, 420],     # fixed
                [676, 427],     # fixed
                [678, 434],     # fixed
                [667, 416],
                [672, 413],
                [677, 416],     # fixed
                [683, 419],     # fixed
                [676, 424],
                [676, 421],
                [674, 417],
                [670, 418],
                [674, 419],     # fixed
                [678, 420],     # fixed
                [674, 408],
                [674, 412],
                [672, 394],
                [668, 385],
                [667, 376],
                [660, 363],
                [662, 355],
                [652, 342],
                [647, 332],
                [642, 322],
                [637, 313],
                [632, 304],
                [626, 294],
                [621, 285],
                [614, 275],
                [608, 265]]

initial_xy = measurements[0]

dt = 1.0/24.0   #0.04166666667

# For EKF, we will need [ x, y, heading (h), velocity, yaw rate ]
x = matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]]) # initial state (location and velocity)

# For EKF, I don't think we need this matrix
u = matrix([[0.], [0.], [0.], [0.]]) # external motion

# For EKF, this could start as a 5x5 matrix for 1000.0 on the diagonal, or perhaps the one here.
P =  matrix([[0., 0., 0., 0.],
             [0., 0., 0., 0.],
             [0., 0., 1000., 0.],
             [0., 0., 0., 1000.]]) # initial uncertainty

# For EKF, we should use the Movement/Prediction jacobian matrix from the comments above
# We'll also need Q, the process noice covariance matrix, which we'll need to figure out. See comments above.
F =  matrix([[1., 0., dt, 0.],
             [0., 1., 0., dt],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]) # next state function
Q = "something here"

# For EKF, this needs to be the Measurement jacobian from the comments above
H =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.]]) # measurement function

# For EKF, this needs to be just a 4x4 or 5x5 identity matrix (leaning towards 4x4)
R =  matrix([[0.1, 0.],
             [0., 0.1]]) # measurement uncertainty 2x2 (0.1)

I =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]) # identity matrix


extended_kalman_filter(x, P)


