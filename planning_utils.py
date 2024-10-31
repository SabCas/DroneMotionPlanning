from enum import Enum
from queue import PriorityQueue
import numpy as np
import math
import random
import networkx as nx
from matplotlib import pyplot as plt


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))
    print(f"Grid size: {north_size} x {east_size}")

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    
    NORTH_WEST = (-1, -1, math.sqrt(2))
    NORTH_EAST = (-1, 1, math.sqrt(2))
    SOUTH_WEST = (1, -1, math.sqrt(2))
    SOUTH_EAST = (1, 1, math.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
        
    if (x-1 < 0) or (y-1 < 0) or grid[x-1, y-1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x-1 < 0) or (y+1 > m) or grid[x-1, y+1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x+1 > n) or (y+1 > m) or grid[x+1, y+1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if (x+1 > n) or (y-1 < 0) or grid[x+1, y-1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)

    return valid_actions


def a_star(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, grid, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path, grid):
    pruned_path = [p for p in path]
    
    i = 0
    while i < (len(pruned_path) - 2):
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        if collinearity_check(p1, p2, p3, grid):
            pruned_path.remove(pruned_path[i+1])
        else:
            i +=1
    return pruned_path



""" Impelementing RRT* algorithm """

class RRTStar:
    def __init__(self, start, goal, grid, max_iter=2000, step_size=5, radius=4):
        self.G = nx.Graph()
        self.G.add_node(start, pos=start, cost=0)
        self.start = start
        self.goal = goal
        self.grid = grid
        self.max_iter = max_iter
        self.step_size = step_size
        self.radius = radius

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.random_sample()
            nearest_node = min(self.G.nodes(), 
                key=lambda n: self.distance(self.G.nodes[n]['pos'], rand_point))
            new_point = self.steer(self.G.nodes[nearest_node]['pos'], rand_point)

            if self.check_collision(self.G.nodes[nearest_node]['pos'], new_point):
                # Find nearby nodes
                near_nodes = [n for n in self.G.nodes() 
                    if self.distance(self.G.nodes[n]['pos'], new_point) < self.radius]
                
                # Find best parent
                min_cost = float('inf')
                best_parent = nearest_node
                for near_node in near_nodes:
                    potential_cost = (self.G.nodes[near_node]['cost'] + 
                                   self.distance(self.G.nodes[near_node]['pos'], new_point))
                    if potential_cost < min_cost and self.check_collision(self.G.nodes[near_node]['pos'], new_point):
                        min_cost = potential_cost
                        best_parent = near_node

                # Add new node
                self.G.add_node(new_point, pos=new_point, cost=min_cost)
                self.G.add_edge(best_parent, new_point, 
                    weight=self.distance(self.G.nodes[best_parent]['pos'], new_point))

                # Rewire
                for near_node in near_nodes:
                    potential_cost = (min_cost + 
                        self.distance(new_point, self.G.nodes[near_node]['pos']))
                    if potential_cost < self.G.nodes[near_node]['cost'] and self.check_collision(new_point, self.G.nodes[near_node]['pos']):
                        old_parent = list(self.G.predecessors(near_node))[0]
                        self.G.remove_edge(old_parent, near_node)
                        self.G.add_edge(new_point, near_node, 
                            weight=self.distance(new_point, self.G.nodes[near_node]['pos']))
                        self.G.nodes[near_node]['cost'] = potential_cost

                if self.distance(new_point, self.goal) < self.step_size:
                    self.G.add_node(self.goal, pos=self.goal, 
                        cost=min_cost + self.distance(new_point, self.goal))
                    self.G.add_edge(new_point, self.goal, 
                        weight=self.distance(new_point, self.goal))
                    print(f"Goal node connected at {self.goal}")
                    break

        if self.goal in self.G:
            return nx.shortest_path(self.G, list(self.G.nodes())[0], self.goal)
    

    def random_sample(self):
        """Generate a random point with bias to goal"""
        if random.random() < 0.05:  # 5% chance to select goal
            return self.goal
        return (random.randint(0, self.grid.shape[0] - 1),
                random.randint(0, self.grid.shape[1] - 1))


    def nearest_node(self, point):
        """Find the nearest node in the tree to the random point."""
        return min(self.tree, key=lambda node: self.distance(node.point, point))

    def steer(self, from_pos, to_pos):
        """Generate a new position in the direction of the random position."""
        direction = np.array(to_pos) - np.array(from_pos)
        length = np.linalg.norm(direction)
        
        if length > self.step_size:
            direction = (direction / length) * self.step_size
        
        new_pos = (from_pos[0] + direction[0], from_pos[1] + direction[1])
        return new_pos


    def check_collision(self, from_pos, to_pos):
        return not self.line_intersects_obstacle(from_pos, to_pos)

    def line_intersects_obstacle(self, from_point, to_point):
        """
        Check if the line from the start point to the end point intersects any obstacles in the grid.

        """
        x0, y0 = from_point
        x1, y1 = to_point
        dx = x1 - x0
        dy = y1 - y0

        steps = int(max(abs(dx), abs(dy)))
        if steps == 0:
            return False
    
        for i in range(steps + 1):
            x = int(x0 + (dx * i / steps))
            y = int(y0 + (dy * i / steps))

            if self.is_in_obstacle(x, y):
                return True  # Collision detected

        return False  

    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))
    
    def is_in_obstacle(self, x, y):
        """Check if point is in obstacle"""
        if x < 0 or x >= self.grid.shape[0] or y < 0 or y >= self.grid.shape[1]:
            return True
        return self.grid[int(x), int(y)] == 1
    

    def plot_graph(self):
        plt.figure(figsize=(12, 12))
        pos = {node: (self.G.nodes[node]['pos'][1], self.G.nodes[node]['pos'][0]) for node in self.G.nodes()}
        
        # Draw grid and full tree
        plt.imshow(self.grid, origin='lower')
        nx.draw(self.G, pos, node_size=20, node_color='b', edge_color='g', alpha=0.3)
        
        # Draw optimal path in red with thicker lines
        path = nx.shortest_path(self.G, self.start, self.goal)
        path_edges = list(zip(path[:-1], path[1:]))
        nx.draw_networkx_edges(self.G, pos, edgelist=path_edges, edge_color='r', width=2)
        
        # Draw start and goal
        start_pos = (self.start[1], self.start[0])
        goal_pos = (self.goal[1], self.goal[0])
        plt.plot(start_pos[0], start_pos[1], 'go', markersize=15, label='Start')
        plt.plot(goal_pos[0], goal_pos[1], 'ro', markersize=15, label='Goal')
        
        plt.legend()
        plt.grid(True)
        plt.show()


        




                                        
        