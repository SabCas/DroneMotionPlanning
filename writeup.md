### Summary of `motion_planning.py` and `planning_utils.py`

#### `motion_planning.py`

The `motion_planning.py` file is responsible for managing the drone's path planning process. The key function, `plan_path()`, works as follows:

1. **Initialization**: It sets the target altitude and defines a safety distance.
2. **Obstacle Data**: Reads collider data from a CSV file to create a grid that represents the environment.
3. **Grid Creation**: Calls the `create_grid()` function from `planning_utils.py`, which constructs a grid based on obstacle positions at a specific altitude and includes a safety margin.
4. **Start and Goal Positions**: Defines the starting and goal positions on the grid. The goal can be modified to target a specific latitude/longitude.
5. **Pathfinding**: Utilizes the A* algorithm (implemented in `planning_utils.py`) to find a path from the start to the goal position, taking into account valid actions and obstacles.
6. **Waypoints**: Converts the resulting path into waypoints that the drone can follow and sends these waypoints for visualization.

#### `planning_utils.py`

This file contains essential utilities for path planning:

1. **Grid Creation**: The `create_grid(data, drone_altitude, safety_distance)` function generates a 2D grid based on obstacle data, marking free space and obstacles for the drone's environment.
2. **Action Enumeration**: The `Action` enum defines possible movements (North, South, East, West) along with their associated costs.
3. **Valid Actions**: The `valid_actions(grid, current_node)` function checks which actions are permissible at the current grid position, avoiding obstacles and grid boundaries.
4. **A* Algorithm**: The `a_star(grid, h, start, goal)` function implements the A* search algorithm to determine the optimal path from the starting point to the goal based on the grid and heuristic calculations.
5. **Heuristic Function**: The `heuristic(position, goal_position)` function calculates the Euclidean distance between the current and goal positions, guiding the A* algorithm in path selection.

### Conclusion

Together, `motion_planning.py` and `planning_utils.py` enable the drone to navigate its environment safely and efficiently by identifying obstacles, determining valid movements, and finding optimal paths to desired locations.

## RRT* Path Planning Implementation
### Overview
I implemented RRT* (Rapidly-exploring Random Tree Star) as an advanced motion planning algorithm that generates optimal paths while efficiently exploring the configuration space. The implementation builds on the core RRT concept by adding cost optimization and tree rewiring capabilities.

#### Key Components
The Node class tracks positions, parent relationships, and accumulated path costs - essential for RRT*'s optimization features. 
Node Structure
```python	
class Node:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent
        self.cost = 0
```



#### Core Algorithm Features
1. **Intelligent Sampling**
   - Random point generation across configuration space
   - 5% goal bias for efficient goal-directed exploration

2. **Tree Growth**
   - Nearest neighbor search
   - Controlled node expansion using step_size parameter
   - Collision checking for obstacle avoidance

3. **Path Optimization**
   - Near-neighbor radius search
   - Cost-based parent selection
   - Dynamic tree rewiring for path improvement

4. **Goal Connection**
   - Goal proximity checking
   - Path retracing from goal to start

#### Technical Implementation
The RRT* planner maintains a growing tree structure, continuously optimizing paths as new nodes are added. The implementation ensures:

- Probabilistic completeness
-  Asymptotic optimality
- Efficient space exploration
- Smooth path generation

#### Results
The algorithm successfully generates collision-free paths while maintaining the optimal balance between exploration and exploitation. 