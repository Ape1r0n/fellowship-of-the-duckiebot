# Duckietown Path Planning 🦆

[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/downloads/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://www.ros.org/)
[![Docker](https://img.shields.io/badge/Docker-20.10%2B-blue.svg)](https://www.docker.com/)
[![NumPy](https://img.shields.io/badge/NumPy-1.23.5-blue.svg)](https://numpy.org/)
[![YOLOv5](https://img.shields.io/badge/YOLOv5-6.2-yellow.svg)](https://github.com/ultralytics/yolov5)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Fellowship of the 🦆
**ETH Zurich Self-Driving Cars with Duckietown Course**  
*Final Project - Software Engineering Practical Course*  
*Kutaisi International University*

**Supervised by:**
- Prof. Walter F. Tichy
- TA: Mikheil Tenieshvili

## Project Overview

This project implements the D* Lite algorithm for optimal path planning in the Duckietown environment. The algorithm allows a Duckiebot to navigate efficiently from one node (red strip/stop sign) to any other node in the Duckietown map, dynamically adapting to changes in the environment.

## D* Lite Algorithm Explained

D* Lite is a pathfinding algorithm particularly well-suited for autonomous robots navigating in partially known or changing environments. It combines the efficiency of A* with the replanning capabilities needed for dynamic environments.

### Intuitive Understanding

At its core, D* Lite works by:

1. **Planning Backwards**: Unlike A*, D* Lite plans from the goal to the start, which makes replanning efficient when the robot moves.
2. **Incremental Search**: It reuses information from previous searches to avoid recomputing the entire path when small changes occur.
3. **Adaptive**: When the robot discovers new obstacles or changes in the environment, D* Lite efficiently updates only the affected parts of the path.

### Key Components and Methods

#### Data Structures
- **g(s)**: The current cost estimate from state s to the goal
- **rhs(s)**: One-step lookahead estimate (helps detect inconsistencies)
- **Priority Queue U**: Stores states to process, ordered by priority keys
- **km**: A cumulative modifier to handle the heuristic changes when the robot moves

#### Core Methods

**1. CalculateKey**
```
CalculateKey(s):
    return [min(g(s), rhs(s)) + h(sstart, s) + km, min(g(s), rhs(s))]
```
*This method computes a two-component priority key for each state, which determines the order in which states are processed. The first component includes the heuristic to guide the search, while the second breaks ties.*

**2. Initialize**
```
Initialize():
    U ← ∅                   # Empty priority queue
    km ← 0                  # Initialize heuristic modifier
    For all s ∈ S:          # Set all states to infinity
        rhs(s) ← ∞
        g(s) ← ∞
    rhs(sgoal) ← 0          # Goal has zero cost to itself
    Insert sgoal into U     # Start expanding from the goal
```
*The initialization sets up the search by setting the goal's cost to zero and placing it in the priority queue as the starting point for the backwards search.*

**3. UpdateVertex**
```
UpdateVertex(u):
    if u ≠ sgoal then
        rhs(u) ← min(c(u, s') + g(s')) for all s' ∈ Succ(u)
    if u ∈ U:
        Remove u from U
    if g(u) ≠ rhs(u):
        Insert u into U with key CalculateKey(u)
```
*This function updates the one-step lookahead value of a vertex based on its successors. If the vertex becomes inconsistent (g ≠ rhs), it's placed in the priority queue for processing.*

**4. ComputeShortestPath**
```
ComputeShortestPath():
    while U.TopKey() < CalculateKey(sstart) or rhs(sstart) ≠ g(sstart):
        u ← U.Top()         # Get state with minimum key
        Remove u from U
        if g(u) > rhs(u):   # If u is overconsistent
            g(u) ← rhs(u)   # Make it consistent
            For all predecessors p of u:
                UpdateVertex(p)  # Update affected states
        else:               # If u is underconsistent
            g(u) �� ∞        # Make it overconsistent
            UpdateVertex(u) and all predecessors
```
*The main algorithm that processes states from the priority queue until the start state becomes consistent or the queue is empty. It resolves inconsistencies and propagates changes through the graph.*

**5. MainLoop**
```
MainLoop():
    slast ← sstart          # Remember last position
    Initialize()
    ComputeShortestPath()
    while sstart ≠ sgoal:   # Until we reach the goal
        if g(sstart) = ∞:   # No path exists
            return failure
        sstart ← best next state  # Move to best neighbor
        Scan for changed edges
        if changes detected:
            km ← km + h(slast, sstart)  # Update heuristic modifier
            slast ← sstart
            Update costs of changed edges
            ComputeShortestPath()  # Replan if needed
```
*This is the main execution loop that moves the robot along the computed path, detects changes in the environment, and triggers replanning when necessary.*

### Algorithm Flow

1. **Initialization**: Set up the search from the goal to the start.
2. **Initial Plan**: Compute the first complete path.
3. **Execution and Replanning**:
   - The robot follows the path one step at a time
   - It scans for changes in the environment (new obstacles, etc.)
   - If changes are detected, update the affected part of the path
   - Continue until the goal is reached

### Advantages in Duckietown

- **Efficient Replanning**: Perfect for navigating Duckietown where other duckiebots or obstacles may appear
- **Optimality**: Guarantees the shortest path given current knowledge
- **Computational Efficiency**: Does not need to recompute the entire path when small changes occur

## Implementation Details

Our implementation uses a graph-based representation of the Duckietown map, where nodes represent red strips/stop signs and edges represent the paths between them. The D* Lite algorithm is used to find the optimal path between any two nodes in this graph.

### Key Features:

- **Dynamic Path Planning**: Adapts to changing conditions in the Duckietown environment
- **Optimal Path Selection**: Always chooses the most efficient route
- **Integration with Duckiebot**: Seamlessly works with the Duckiebot's perception and control systems
- **Visualization**: Provides real-time visualization of the planned path

## Installation

```bash
# Clone the repository
git clone https://github.com/Ape1r0n/fellowship-of-the-duckiebot.git 
cd fellowship-of-the-duckiebot
python3 Dlite.py
```

## Usage

```python
from Dlite import create_graph, DStarLite

# Create graph from Duckietown map
## provided text file should include weighted directed graph in the following format:
## C(vi, vj) = k where vi and vj are nodes, k is cost between the edge vi -> vj
graph = create_graph("map.txt")

# Initialize D* Lite with start and goal positions
path_planner = DStarLite(start_node, goal_node, graph)

# Get the optimal path
path = path_planner.get_path()
```
