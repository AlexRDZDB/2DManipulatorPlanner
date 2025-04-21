'''
2D Manipulator Class to create a 2D Robotic Arm with three-link and obtain corresponding coordinates

Attributes:
    - LinkLengths: Tuple of floats that specifies the length of the links of the robot. By default, these links are of size 1
    - Name: Name of the robot
    - res: Resolution of the discretization of the robot for calculation of c-space
'''
import numpy as np

class Manipulator2D():
    
    def __init__(self, LinkLengths = (1.0, 1.0, 1.0), name="2DManipulator", res=30):
        self.__LinkLengths = tuple(LinkLengths) # Ensure immutability
        self.__name = name
        self.__currentConfiguration = [self.__res // 2] * len(self.__LinkLengths) # Initialize robot config at 0, 0, 0 rads
        self.__res = res
        self.__cspace = np.zeros((self.__res, self.__res, self.__res), dtype=np.uint8) # Store C-Space

        
        