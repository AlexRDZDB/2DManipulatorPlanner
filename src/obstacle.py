''' Obstacle class to represent an obstacle inside of the C-Space'''
import numpy as np

class Obstacle():
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    
    def containsPoint(self, point):
        x, y = point
        cx, cy = self.center
        return np.hypot(x - cx, y - cy) <= self.radius