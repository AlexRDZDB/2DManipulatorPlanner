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
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter

from .obstacle import Obstacle
from .AStar import computeAStar

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
        
        # Plot obstacles
        for obstacle in self.obstacles:
            origin = obstacle.center
            radius = obstacle.radius
            obstacle_plot = plt.Circle(origin, radius, color ='red', fill=True) # Plots the Circle
            obstacle_border = plt.Circle(origin, radius, color="black", fill=False, linewidth=1.5) # Plots the Stroke
            ax.add_patch(obstacle_plot)
            ax.add_patch(obstacle_border)
        
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

    # Function to generate the robot's configuration space according to the obstacles in it
    def generateCSpace(self, offset=0):
        for i, q1 in enumerate(self.discretizedAngles):
            q = [q1, 0, 0]

            if self.checkCollision(q, 1, offset=offset):
                self.cspace[i,:,:] = 1
                continue
            
            else:
                for j, q2 in enumerate(self.discretizedAngles):
                    q = [q1, q2, 0]
                    if self.checkCollision(q, 2, offset=offset):
                        self.cspace[i,j,:] = 1
                        continue
                    else:
                        for k, q3 in enumerate(self.discretizedAngles):
                            q = [q1,q2,q3]
                            if self.checkCollision(q, 3, offset=offset):
                                self.cspace[i,j,k] = 1
                                continue
    
    # Given a set of angles q, check if there are any collisions with obstacles
    def checkCollision(self, q, linkNo, offset=0):
        positions = self.forwardKinematics(q) # Obtain joint position

        for i in range(len(positions) - 1):
            if i == linkNo:
                return False
            
            start = positions[i] # Load two consecutive joint positions
            end = positions[i + 1]
            # Check if there is collision with obstacles
            for obstacle in self.obstacles:
                if self.intersectsObstacle(start, end, obstacle, offset=offset):
                    return True
        
        return False
    
    # Check whether a given link intersects the obstacle at any point
    def intersectsObstacle(self, start, end, obstacle, offset=0):
        center = np.array(obstacle.center)
        radius = obstacle.radius + offset

        # Convert points to numpy arrays for vector operations
        start = np.array(start)
        end = np.array(end)

        # Establish vector from start to end
        direction = end - start 
        
        # Vector from circle center to the start point
        f = start - center

        # Coefficients for quadratic equation
        a = np.dot(direction, direction)
        b = 2 * np.dot(f,direction)
        c = np.dot(f,f) - radius**2

        # Compute discriminant
        discriminant = b**2 - 4*a*c

        if discriminant < 0:
            # No real roots: line doesn't intersect circle
            return False
        else:
            # Compute the two roots (quadratic formula)
            discriminant_sqrt = np.sqrt(discriminant)

            t1 = (-b - discriminant_sqrt) / (2 * a)
            t2 = (-b + discriminant_sqrt) / (2 * a)

            # Check if either root lies within the segment bounds [0, 1]
            if (0 <= t1 <= 1) or (0 <= t2 <= 1):
                return True

            return False

    # Function to add new obstacles to the working space
    def addObstacle(self, obstacle):
        self.obstacles.append(obstacle)

    # Plot CSpace
    def plotCSpace(self):
        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')

        filled = self.cspace.astype(bool)

        colors = np.empty(self.cspace.shape, dtype=object)
        colors[self.cspace == 1] = 'red'
        colors[self.cspace == 0] = 'none'

        ax.voxels(filled, facecolors=colors, edgecolor='k', linewidth=0.2)

        # Use actual angle values as ticks (limit to avoid clutter)
        tick_indices = np.linspace(0, len(self.discretizedAngles) - 1, 5, dtype=int)
        angle_ticks = np.round(self.discretizedAngles[tick_indices], 2)

        ax.set_xticks(tick_indices)
        ax.set_xticklabels(angle_ticks)
        ax.set_yticks(tick_indices)
        ax.set_yticklabels(angle_ticks)
        ax.set_zticks(tick_indices)
        ax.set_zticklabels(angle_ticks)
        ax.set_xlabel("Joint 1")
        ax.set_ylabel("Joint 2")
        ax.set_zlabel("Joint 3")
        ax.set_title(f"{self.name} Joint Space Configuration")
        plt.tight_layout()
        plt.show()

    # Inverse Kinematics for fixed end-effector orientation
    def inverseKinematics(self, position, phi=0.0):
        x, y = position
        L1, L2, L3 = self.LinkLengths

        # Compute wrist position
        x_wrist = x - L3 * np.cos(phi)
        y_wrist = y - L3 * np.sin(phi)

        # Distance from origin to wrist
        r = np.hypot(x_wrist, y_wrist)

        # Check reachability
        if r > (L1 + L2) or r < abs(L1 - L2):
            print("Target not reachable")
            return None

        # Inverse Kinematics Calculation for q1 and q2
        # Law of Cosines for q2
        cos_q2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
        sin_q2_pos = np.sqrt(1 - cos_q2**2)
        sin_q2_neg = -sin_q2_pos

        q2_options = [np.arctan2(sin_q2_pos, cos_q2), np.arctan2(sin_q2_neg, cos_q2)]
        solutions = []

        for q2 in q2_options:
            # k1, k2 are the intermediate link values
            k1 = L1 + L2 * np.cos(q2)
            k2 = L2 * np.sin(q2)

            q1 = np.arctan2(y_wrist, x_wrist) - np.arctan2(k2, k1)
            q3 = phi - q1 - q2

            # Normalize angles to [-pi, pi]
            q1 = np.arctan2(np.sin(q1), np.cos(q1))
            q2 = np.arctan2(np.sin(q2), np.cos(q2))
            q3 = np.arctan2(np.sin(q3), np.cos(q3))

            solutions.append([q1, q2, q3])

        return solutions

    # Simulate motion between poses. firstPath value forces return of only one path
    def motionPlanner(self, poseA, poseB, phiA = 0.0, phiB =0.0, firstPath=False):
        # Obtain IK for pointA and pointB
        ik_poseA = self.inverseKinematics(poseA, phiA)
        ik_poseB = self.inverseKinematics(poseB, phiB)

        valid_paths = []
        for configA in ik_poseA:
            for configB in ik_poseB:
                # Transform joint angles to discretized values
                discreteA = [self.getIndexFromAngle(configA[0]),
                             self.getIndexFromAngle(configA[1]),
                             self.getIndexFromAngle(configA[2])]
                
                discreteB = [self.getIndexFromAngle(configB[0]),
                             self.getIndexFromAngle(configB[1]),
                             self.getIndexFromAngle(configB[2])]
                                
                # Run A-star algorithm to determine if there is a valid path between the two
                path = computeAStar(discreteA, discreteB, self.cspace, self.res)

                if path and firstPath:
                    return path
                
                valid_paths.append(path)
        
        return valid_paths
    
    # Animate a motion
    def animateRobot(self, path, interval=250, save=False, name="robot_motion.gif"):
        fig, ax = plt.subplots()

        start_point = path[0]
        end_point = path[-1]

        start_angles = [self.discretizedAngles[start_point[0]],
                        self.discretizedAngles[start_point[1]],
                        self.discretizedAngles[start_point[2]]]
        
        end_angles = [self.discretizedAngles[end_point[0]],
                    self.discretizedAngles[end_point[1]],
                    self.discretizedAngles[end_point[2]]]
        
        start_config = self.forwardKinematics(start_angles)
        end_config = self.forwardKinematics(end_angles)


        ax.set_xlim(-sum(self.LinkLengths) - 1, sum(self.LinkLengths) + 1)
        ax.set_ylim(-sum(self.LinkLengths) - 1, sum(self.LinkLengths) + 1)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        line, = ax.plot([], [], color="blue", marker="o",markersize=6,lw=3)
        reach_circle = plt.Circle((0,0), sum(self.LinkLengths), color="gray", fill=False, linestyle='dotted', linewidth=1.5)
        ax.add_patch(reach_circle)

        start_circle = plt.Circle((start_config[-1]), 0.125, color = "green", fill=False, linewidth = 1.5)
        end_circle = plt.Circle((end_config[-1]), 0.125, color = "darkgreen", fill=True, linewidth = 1.5)

        ax.add_patch(start_circle)
        ax.add_patch(end_circle)

            # Plot obstacles once
        for obstacle in self.obstacles:
            origin = obstacle.center
            radius = obstacle.radius
            obstacle_plot = plt.Circle(origin, radius, color='red', fill=True)
            obstacle_border = plt.Circle(origin, radius, color="black", fill=False, linewidth=1.5)
            ax.add_patch(obstacle_plot)
            ax.add_patch(obstacle_border)

        def init():
            line.set_data([], [])
            return line,
    
        def update(frame):
            
            angles = [self.discretizedAngles[frame[0]],
                    self.discretizedAngles[frame[1]],
                    self.discretizedAngles[frame[2]]]
            
            self.updatePositions(angles)

            positions = self.forwardKinematics()

            x_coords = [p[0] for p in positions]
            y_coords = [p[1] for p in positions]

            line.set_data(x_coords, y_coords)
            return line,

        
        anim = animation.FuncAnimation(
            fig, update, frames=path, init_func=init,
            interval=interval, blit=False, repeat=False
            )
        if save:
            writer = PillowWriter(fps=1000 // interval)
            anim.save(name, writer=writer)
        
        plt.show()
    