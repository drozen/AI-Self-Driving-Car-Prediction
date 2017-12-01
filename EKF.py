__author__ = 'agmadi'

import sys
import turtle
from numpy import matrix, eye, diag, abs, sin, cos, pi, linalg

class EKF(object):
    """
    Extended Kalman Filter
    """

    def __init__( self, ranger, extendedData, dataMinX, dataMinY, dataMaxX, dataMaxY, predictFrames, yawLimit, debug ):
        """
        Initialze the filter with all needed data
        :rtype : None
        :type ranger: RangeFinder
        :type extendedData: list
        :type dataMinX: float
        :type dataMinY: float
        :type dataMaxX: float
        :type dataMaxY: float
        :type predictFrames: int
        :type yawLimit: float
        :type debug: bool
        """
        self.ranger = ranger
        self.extendedData = extendedData
        self.dataMinX = dataMinX
        self.dataMinY = dataMinY
        self.dataMaxX = dataMaxX
        self.dataMaxY = dataMaxY
        self.predictFrames = predictFrames
        self.yawLimit = yawLimit
        self.debug = debug

        self.dt = 1.0
        self.initial_data = self.extendedData[0]
        self.I = eye( 5 ) # identity matrix

        # measurement covariance
        self.R = matrix([
            [0.1, 0., 0., 0., 0.],
            [0., 0.1, 0., 0., 0.],
            [0., 0., 0.1, 0., 0.],
            [0., 0., 0., 0.1, 0.],
            [0., 0., 0., 0., 0.1],
        ])

        # the process noice covariance matrix
        self.Q = diag([1., 1., 1., 1., 1.])

        self.predicted = None
        self.x = None
        self.P = None
        self.PLearned = None

        self.reset()

    def reset( self ):
        """
        Reset filter parameters and output data to get ready for a run.
        :rtype : None
        """
        self.predicted = []

        # we will need [ x, y, heading (h), velocity (v), yaw rate (yr) ]
        self.x = matrix([
            [self.initial_data[0],
             self.initial_data[1],
             self.initial_data[2],
             self.initial_data[3],
             self.initial_data[4]]
        ]).T # initial state (location, heading, velocity, and yaw rate)

        # this could start as a 5x5 matrix with 1000.0 on the diagonal.
        self.P =  diag( [1000.0, 1000.0, 1000.0, 1000.0, 1000.0] ) # initial uncertainty

    def runFilter( self ):
        """
        Run the EKF and return a list of predicted coordinates.
        :rtype : list
        :return: a list of predicted coordinates.
        """

        self.reset()

        # if we have a learned uncertainty from a previous run, then use it
        if self.PLearned is not None:
            # ignore the x,y positions though because we have no idea where the new data ends.
            self.P[2] = self.PLearned[2]
            self.P[3] = self.PLearned[3]
            self.P[4] = self.PLearned[4]

        # run through all data and return final state vector and uncertainty.
        stateVector, PVector = self.extended_kalman_filter( self.extendedData, self.x, self.P, self.Q, self.R, self.I )

        if self.debug:
            print 'X1=', stateVector

        # we get the median velocity to send into our predictor. if we don't do this, the velocity will
        # end up being closer to the velocity that was calculated near the end of the run.  experimentation
        # showed that to be a bad idea since sometimes the last few points were very close together because
        # of the robot's bouncing or measurement noice and the predictions ended up all clumped together.
        stateVector[3] = self.ranger.getMediaVelocity()
        if self.debug:
            print 'Median Velocity is', self.ranger.getMediaVelocity()
            print 'min', self.dataMinX, self.dataMinY
            print 'max', self.dataMaxX, self.dataMaxY

        # for self.predictFrames times call predictor.
        framesCount = 0
        while framesCount < self.predictFrames:
            # fix yaw rate since it's so very noisy
            # not doing this step always caused the predictions to go in a circle.
            # the robot measurements are just too noisy not to do this step.
            # a yaw limit of 0 turns this off
            if self.yawLimit > 0:
                if stateVector[4] < -self.yawLimit:
                    stateVector[4] = -self.yawLimit
                elif stateVector[4] > self.yawLimit:
                    stateVector[4] = self.yawLimit

            # call the predictor function to get the next prediction
            stateVector = self.predict_EKF( stateVector )
            x, y, heading, velocity, yawRate = stateVector
            if self.debug:
                print x, y

            # make sure we don't hit a wall before calling next step
            # test if the predicted point is at a wall or beyond it and compensate accordingly.
            # check x and y against walls ( dataMinX, dataMinY, dataMaxX, dataMaxY )
            # compensate for the thickness of the robot by adjust the walls by 10 pixels on each side.
            if x <= self.dataMinX + 10:
                stateVector[2] = self.ranger.angle_trunc( pi - stateVector[2] ) # bounce it off left wall
                stateVector[0] = self.dataMinX + 10
            elif x >= self.dataMaxX - 10:
                stateVector[2] = self.ranger.angle_trunc( pi - stateVector[2] ) # bounce it off right wall
                stateVector[0] = self.dataMaxX - 10
            elif y <= self.dataMinY + 10:
                stateVector[2] = self.ranger.angle_trunc( -stateVector[2] ) # bounce it off top wall
                stateVector[1] = self.dataMinY + 10
            elif y >= self.dataMaxY - 10:
                stateVector[2] = self.ranger.angle_trunc( -stateVector[2] ) # bounce it off bottom wall
                stateVector[1] = self.dataMaxY - 10
            else:
                # add the prediction to the prediction list
                self.predicted.append( ( x[0,0], y[0,0], heading[0,0] ) )
                framesCount += 1

        if self.debug:
            print 'X2=', stateVector
        self.PLearned = PVector

        return self.predicted

    def visualize( self ):
        """
        Show a screen with the predictions drawn on it.
        :rtype : None
        """
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
        boundsMinX = self.dataMinX - xOffset
        boundsMaxX = self.dataMaxX - xOffset
        boundsMinY = self.dataMinY - yOffset
        boundsMaxY = self.dataMaxY - yOffset
        bounds.goto(boundsMinX, boundsMinY)
        bounds.pendown()
        bounds.goto(boundsMaxX, boundsMinY)
        bounds.goto(boundsMaxX, boundsMaxY)
        bounds.goto(boundsMinX, boundsMaxY)
        bounds.goto(boundsMinX, boundsMinY)

        hexbug_visual.color('red')
        startAt = len(self.extendedData)-120
        if startAt < 0:
            startAt = 0
        for i in self.extendedData[startAt:]:
            hexbug_visual.setheading( i[2] * 180 / pi )
            hexbug_visual.goto( i[0] - xOffset, i[1] - yOffset)
            hexbug_visual.stamp()

        hexbug_visual.color('blue')
        for i in self.predicted:
            hexbug_visual.setheading( i[2] * 180 / pi )
            hexbug_visual.goto( i[0] - xOffset, i[1] - yOffset)
            hexbug_visual.stamp()

        # pause to allow the user to see the output
        print 'Press ENTER to end'
        sys.stdin.readline()

    def vectorize( self, measurementVector ):
        """
        Turn a measurement vector into a numpy matrix to be used in calculations
        :rtype : matrix
        :param measurementVector: float: x, float: y, float: h, float: v, float: yr
        :return: matrix containing x, y, velocity, yawRate.
        """
        x, y, heading, velocity, yawRate = measurementVector
        return matrix( [ x, y, heading, velocity, yawRate ] ).T

    def predict_EKF( self, stateVector ):
        """
        Predict the next position given current position vector
        :rtype : matrix
        :param stateVector: float: x, float: y, float: h, float: v, float: yr
        :return: matrix containing newly computed x, y, velocity, heading, yawrate
        """
        x, y, heading, velocity, yawRate = stateVector
        if abs( yawRate ) < 0.0001:
            #Driving straight - avoid division by zero (yr < 0.0001)
            x = x + velocity * self.dt * cos( heading )
            y = y + velocity * self.dt * sin( heading )
            heading = heading
            velocity = velocity
            yawRate = 0.0000001 # avoid numerical issues in Jacobian calculations later
        else:
            # Driving not so straight
            x = x + ( velocity / yawRate ) * ( sin( yawRate * self.dt + heading ) - sin( heading ) )
            y = y + ( velocity / yawRate ) * ( -cos( yawRate * self.dt +heading ) + cos( heading ) )
            heading = ( heading + yawRate * self.dt + pi ) % (2.0 * pi) - pi
            velocity = velocity
            yawRate = yawRate

        return matrix( [ float(x), float(y), float(heading), float(velocity), float(yawRate) ] ).T

    def createPredictionJacobian( self, stateVector ):
        """
        Create a Jacobian matrix that will be used in computing the uncertainty
        :rtype : matrix
        :param stateVector: float: x, float: y, float: h, float: v, float: yr
        :return: Prediction Jacobian matrix
        """
        x, y, heading, velocity, yawRate = stateVector
        a_1_3 = float( ( velocity / yawRate ) * ( cos( yawRate * self.dt + heading ) - cos( heading ) ) )
        a_1_4 = float( ( 1.0 / yawRate ) * ( sin( yawRate * self.dt + heading ) - sin( heading ) ) )
        a_1_5 = float( ( self.dt * velocity / yawRate ) * cos( yawRate * self.dt + heading ) -
                       ( velocity / yawRate**2 ) * ( sin( yawRate * self.dt + heading ) - sin( heading ) ) )
        a_2_3 = float( ( velocity / yawRate ) * ( sin( yawRate * self.dt + heading ) - sin( heading ) ) )
        a_2_4 = float( ( 1.0 / yawRate ) * ( -cos( yawRate * self.dt + heading ) + cos( heading ) ) )
        a_2_5 = float( ( self.dt * velocity / yawRate ) * sin( yawRate * self.dt + heading ) -
                       ( velocity / yawRate**2 ) * ( -cos( yawRate * self.dt + heading ) + cos( heading ) ) )

        return matrix( [
            [1.0, 0.0, a_1_3, a_1_4, a_1_5],
            [0.0, 1.0, a_2_3, a_2_4, a_2_5],
            [0.0, 0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ])

    def createMeasurementJacobian( self, measurementVector ):
        """
        Create a Jacobian matrix that will be used in updating the filter with measurement data
        :rtype : matrix
        :param measurementVector: float: x, float: y, float: h, float: v, float: yr
        :return: Measurement Jacobian matrix
        """
        x, y, heading, velocity, yawRate = measurementVector
        if x == -1 and y == -1:
            # bad data so don't use it
            # by setting the top 2 lines to all zeros, we ignore the
            # bad measurement data here but still account for the velocity and yaw rate.
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

    def extended_kalman_filter( self, extendedData, x, P, Q, R, I ):
        """
        Run the EKF on all of the data
        :rtype : matrix, matrix
        :param extendedData: A list of all measurement data containing ( x, y, heading, velocity, yaw rate )
        :param x: Initial state
        :param P: Uncertainty/covariance
        :param Q: Process noice covariance
        :param R: Measurement covariance
        :param I: Identity matrix
        :type extendedData: list
        :type x: numpy.matrixlib.defmatrix.matrix
        :type P: numpy.ndarray
        :type Q: numpy.ndarray
        :type R: numpy.ndarray
        :type I: numpy.ndarray
        :return: Final prediction matrix and final uncertainty matrix
        """

        # go through all the data and perform the measurement and prediction steps
        for n in xrange( len( extendedData ) ):

            # measurement update for EKF
            JacobianH = self.createMeasurementJacobian( extendedData[ n ] )
            hOFx = matrix([
                [ float( x[0] ) ],
                [ float( x[1] ) ],
                [ float( x[2] ) ],
                [ float( x[3] ) ],
                [ float( x[4] ) ]
            ])
            Z = self.vectorize( extendedData[ n ] )
            y = Z - hOFx
            S = JacobianH * P * JacobianH.T + R
            K = ( P * JacobianH.T ) * linalg.inv( S )
            x = x + ( K * y )
            P = I - ( ( K * JacobianH ) * P )

            # prediction for EKF
            x = self.predict_EKF( x )
            JacobianF = self.createPredictionJacobian( x )
            P = JacobianF * P * JacobianF.T + Q

        return x, P

if __name__ == '__main__':
    print "EKF module must be imported"
