# Using code from homework #2 as a first test
#
#   I tried just plugging in a few seconds of the data from the training file.
#   This failed miserably.  The Kalman filter is meant to only work on linear
#   motion, so it keeps predicting the robot will keep going in the same general
#   direction that it started.
#
#   If we want to continue using a Kalman filter, we'll have to first go through
#   the data and figure out everywhere the robot changes directions, and restart
#   the filter at those points.
#
#   However, we should probably look into using an Extended Kalman Filter instead.
#   The EKF has the ability to work on non-linear data, so we shouldn't have to
#   figure out where the walls are before running the data through.  We'll only need
#   to know the wall locations after the filter is complete and we start doing our
#   extended, 60 frame, predictions.
#
#

from math import *

class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

def kalman_filter(x, P):
    for n in range(len(measurements)):
        
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        
        # measurement update
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
    
    print 'x= '
    x.show()
    print 'P= '
    P.show()

########################################

print "### 4-dimensional example ###"

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
                [658, 399],
                [649, 400],
                [657, 404],
                [661, 407],
                [670, 420],
                [672, 413],
                [667, 416],
                [672, 413],
                [676, 424],
                [676, 421],
                [674, 417],
                [670, 418],
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

dt = 0.04166666667

x = matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]]) # initial state (location and velocity)
u = matrix([[0.], [0.], [0.], [0.]]) # external motion

P =  matrix([[0., 0., 0., 0.],
             [0., 0., 0., 0.],
             [0., 0., 1000., 0.],
             [0., 0., 0., 1000.]]) # initial uncertainty
F =  matrix([[1., 0., dt, 0.],
             [0., 1., 0., dt],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]) # next state function
H =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.]]) # measurement function
R =  matrix([[0.1, 0.],
             [0., 0.1]]) # measurement uncertainty 2x2 (0.1)
I =  matrix([[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]]) # identity matrix

###### DO NOT MODIFY ANYTHING HERE #######

kalman_filter(x, P)
