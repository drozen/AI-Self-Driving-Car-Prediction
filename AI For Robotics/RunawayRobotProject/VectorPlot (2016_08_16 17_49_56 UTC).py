%matplotlib inline
from functools import wraps
import matplotlib.pyplot as plt
import numpy as np
 
class VectorPlot:
    
    def __init__(self):
        self.x_est = np.array([])
        self.y_est = np.array([])
        self.x_true = np.array([])
        self.y_true = np.array([])
        self.fig = plt.figure()
 
    def make_plot(self):
        plt.quiver(self.x_true[:-1], self.y_true[:-1], self.x_est[:-1]-self.x_true[:-1], self.y_est[:-1]-self.y_true[:-1], scale_units='xy', angles='xy', pivot='tail', color='red', alpha=0.5, scale=1)
        plt.quiver(self.x_true[:-1], self.y_true[:-1], self.x_true[1:]-self.x_true[:-1], self.y_true[1:]-self.y_true[:-1], scale_units='xy', angles='xy', pivot='tail', color='blue', alpha=0.5, scale=1)
        self.fig.canvas.draw()
          
    def add(self, predicted, actual):
        self.x_est = np.append(self.x_est, predicted[0])
        self.y_est = np.append(self.y_est, predicted[1])
        self.x_true = np.append(self.x_true, actual[0])
        self.y_true = np.append(self.y_true, actual[1])
        
def plot_me(func):
    vp = VectorPlot()
    @wraps(func)
    def wrapper(*args, **kwargs):
        actual = args[0]
        predicted, OTHER = func(*args, **kwargs)
        vp.add(predicted, actual)
        plt.clf()
        vp.make_plot()
        return predicted, OTHER
    return wrapper