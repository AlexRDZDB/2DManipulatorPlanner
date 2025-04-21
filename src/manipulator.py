'''
2D Manipulator Class to create a 2D Robotic Arm with three-link and obtain corresponding coordinates

Attributes:
    - LinkLengths: Tuple of floats that specifies the length of the links of the robot. By default, these links are of size 1
    - Name: Name of the robot
    - res: Resolution of the discretization of the robot for calculation of c-space
    - currentQ: Current Joint Angles
    - origin: Origin point of the robot
    - discretizedAngles: np.linspace of all possible discretized angles
'''
import numpy as np
import matplotlib.pyplot as plt

class Manipulator2D():
    
    def __init__(self, LinkLengths = (1.0, 1.0, 1.0), name = "2DManipulator", res = 35):
        # Robot Static State Parameters
        self.origin = (0,0)
        self.LinkLengths = LinkLengths
        self.name = name
        self.res = res
        self.discretizedAngles = np.linspace(-np.pi, np.pi, res)

        # Robot Dynamic State Parameters
        self.currentQ = [0.0, 0.0, 0.0] # stored in rads
        self.currentPos = self.forwardKinematics()
        self.obstacles = []
        self.cspace = np.zeros((self.res, self.res, self.res), dtype=np.uint8)
    
    # Helper Function to obtain the closest discretized angle within the linspace array
    def getIndexFromAngle(self, angle):
        return np.argmin(np.abs(self.discretizedAngles - angle))

    # Function to calculate robot forward kinematics
    def forwardKinematics(self, angles=None):
        ''' Implement forward kinematic calculations and obtain joint positions using linear algebra
        '''
        # With no parameters, the angles used will be the ones stored in currentQ
        if angles is None:
            angles = self.currentQ
        
        # Initialize a transformation matrix to use
        T = np.eye(3)

        # Store positions of each joint
        positions = [(0,0)]

        # For every link
        for i in range(len(self.LinkLengths)):
            # Get length and set angle of each joint
            currLength = self.LinkLengths[i]
            angIdx = self.getIndexFromAngle(angles[i])
            currAng = self.discretizedAngles[angIdx]

            # Transformation Matrix
            transform = np.array([[np.cos(currAng), -np.sin(currAng), currLength * np.cos(currAng)],
                                  [np.sin(currAng), np.cos(currAng), currLength * np.sin(currAng)],
                                  [0, 0, 1]])
            
            # Apply transformation of the link to the global transformation matrix
            T = np.dot(T, transform)

            # Extract positional information of the joint location
            position = (T[0, 2], T[1, 2])
            positions.append(position)
        
        print(positions)
        return positions
    
    # Function to update robot positions according to new input angles
    def updatePositions(self, angles):
        self.currentQ = angles
        self.currentPos = self.forwardKinematics(angles)

    # Function to graph current robot state with matplotlib
    def plotRobot(self, color="blue"):
        # Obtain joint positions
        positions = self.forwardKinematics()

        # Initialize plot
        fig, ax = plt.subplots()
        
         # Extract X and Y coordinates
        x_coords = [p[0] for p in positions]
        y_coords = [p[1] for p in positions]
        
        # Plot links
        ax.plot(x_coords, y_coords, color=color, linewidth=3, marker='o', markerfacecolor="black")
        
        total_length = sum(self.LinkLengths)
        reach_circle = plt.Circle(self.origin, total_length, color='gray', fill=False, linestyle='dotted', linewidth=1.5)
        ax.add_patch(reach_circle)
        
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f"{self.name} - Robot Configuration")

        # Set axis limits
        ax.set_xlim(-total_length - 0.5, total_length + 0.5)
        ax.set_ylim(-total_length - 0.5, total_length + 0.5)

        plt.show()

robot = Manipulator2D()
print("Current Info: ", robot.currentQ, robot.currentPos)
robot.plotRobot()