''' Helper class for AStar algorithm implementation'''
import heapq

# Heuristic function for A* algorithm
def ManhattanDistance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2])

class Node:
    def __init__(self, position, h, cost=0, parent=None):
        self.position = tuple(position)
        self.cost = cost
        self.heuristic = h 
        self.total_cost = self.cost + self.heuristic
        self.parent = parent

    def __lt__(self, other):
        return self.total_cost < other.total_cost
    # Function to get all valid neighbors of a cell
    def getNeighbors(self, map, res, target):
        neighbors = []
        directions = [
            (1,0,0), (-1,0,0),
            (0,1,0), (0,-1,0),
            (0,0,1), (0,0,-1)
        ]

        x,y,z = self.position

        for dx, dy, dz in directions:
            nx, ny, nz = x + dx, y + dy, z + dz

            if 0 <= nx < res and 0 <= ny < res and 0 <= nz < res:
                if map[nx][ny][nz] == 0:
                    neighbor_node = Node(
                        position=(nx,ny,nz),
                        h = ManhattanDistance((nx,ny,nz), target),
                        cost = self.cost + 1,
                        parent = self
                    )
                    neighbors.append(neighbor_node)
        
        return neighbors

def computeAStar(startPoint, endPoint, map, res):
    # Define start node
    startNode = Node(startPoint, ManhattanDistance(startPoint, endPoint))
    endPoint = tuple(endPoint)
    # Initialized open and closed list
    openList = []
    closedList = set()
    nodesToCheck = {}

    heapq.heappush(openList, startNode)
    
    # Start searching
    while openList:
        currNode = heapq.heappop(openList)
        
        # Check if we have reached the desired goal
        if currNode.position == endPoint:
            # Construct path
            path = []
            while currNode:
                path.append(currNode.position)
                currNode = currNode.parent
            # Return path in reverse
            return path[::-1]

        # If not the end goal, add the position to the closed list
        closedList.add(currNode.position)

        # Check all neighbors
        for neighbor in currNode.getNeighbors(map, res, endPoint):
            if neighbor.position in closedList:
                continue

            # Check whether the new cost of the neighbors is more or less than the previous cost
            if neighbor.position in nodesToCheck and nodesToCheck[neighbor.position] <= neighbor.total_cost:
                continue
            
            print(f"Added {neighbor.position}")
            nodesToCheck[neighbor.position] = neighbor.total_cost
            heapq.heappush(openList, neighbor)
    
    return False