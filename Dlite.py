import heapq
import re


def create_graph(filename: str) -> dict:
    '''
    Parses a file to create a weighted directed graph.

    The file should contain lines in the format 'C(node1, node2) = cost',
    where node1 and node2 are node identifiers (e.g., 'v1', 'v2') and cost
    is an integer representing the weight of the edge from node1 to node2.

    Args:
        filename: The path to the file containing the graph data.

    Returns:
        A dictionary representing the graph in the format {node_i: {node_j: cost_j, ...}, ...}.

    Raises:
        FileNotFoundError: If the specified file does not exist.
        ValueError: If a line in the file does not match the expected format.
    '''
    graph: dict = {}
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                match = re.match(r"C\((v\d+), (v\d+)\) = (\d+)", line)
                if match:
                    node_from, node_to, cost_str = match.groups()
                    cost = int(cost_str)
                    if node_from not in graph:
                        graph[node_from] = {}
                    graph[node_from][node_to] = cost
                else:
                    raise ValueError(f"Invalid line format: {line}")
    except FileNotFoundError:
        raise FileNotFoundError(f"File not found: {filename}")
    return graph


class DStarLite:
    """
    Implementation of the D* Lite algorithm for path planning in dynamic environments.
    
    D* Lite efficiently computes the shortest path from a start state to a goal state
    in a weighted directed graph, and can efficiently replan when edge costs change.
    """

    def __init__(self, start: str, goal: str, graph: dict):
        """
        Initialize the D* Lite algorithm.
        
        Args:
            start: The identifier of the start state.
            goal: The identifier of the goal state.
            graph: A dictionary representing the graph in the format
                  {node_i: {node_j: cost_j, ...}, ...}.
        """
        self.graph = graph
        self.s_start = start
        self.s_goal = goal
        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0
        self.s_last = start
        
        for node in self.get_all_nodes():
            self.g[node] = float('inf')
            self.rhs[node] = float('inf')
        
        self.rhs[self.s_goal] = 0
        
        self.insert(self.s_goal, self.calculate_key(self.s_goal))

    def get_all_nodes(self) -> list:
        """
        Get all nodes in the graph.
        
        Returns:
            A list of all node identifiers.
        """
        nodes = set()
        for node, edges in self.graph.items():
            nodes.add(node)
            for successor in edges:
                nodes.add(successor)
        return list(nodes)

    def h(self, s1: str, s2: str) -> float:
        """
        Heuristic function estimating the cost between two states.
        
        The heuristic should be admissible (never overestimate the cost).
        This implementation uses a simple uniform cost of 1 between any nodes
        if they're not the same.
        
        Args:
            s1: First state.
            s2: Second state.
            
        Returns:
            Estimated cost from s1 to s2.
        """
        return 0 if s1 == s2 else 1

    def calculate_key(self, s: str) -> list:
        """
        Calculate the key for a state to be used in the priority queue.
        
        Args:
            s: State to calculate key for.
            
        Returns:
            A list [k1, k2] representing the priority of s.
        """
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km, min(self.g[s], self.rhs[s])]

    def insert(self, s: str, key: list) -> None:
        """
        Insert a state into the priority queue with the given key.
        
        Args:
            s: State to insert.
            key: Priority key for the state.
        """
        heapq.heappush(self.U, (key[0], key[1], s))

    def remove_top(self) -> str:
        """
        Remove and return the top state from the priority queue.
        
        Returns:
            The state with the lowest key.
        """
        return heapq.heappop(self.U)[2]

    def top_key(self) -> list:
        """
        Get the key of the top state in the priority queue without removing it.
        
        Returns:
            The key of the top state, or [float('inf'), float('inf')] if queue is empty.
        """
        if not self.U:
            return [float('inf'), float('inf')]
        return [self.U[0][0], self.U[0][1]]

    def get_successors(self, s: str) -> list:
        """
        Get the successors of a state.
        
        Args:
            s: The state to get successors for.
            
        Returns:
            A list of successor states.
        """
        if s in self.graph:
            return list(self.graph[s].keys())
        return []

    def get_predecessors(self, s: str) -> list:
        """
        Get the predecessors of a state.
        
        Args:
            s: The state to get predecessors for.
            
        Returns:
            A list of predecessor states.
        """
        predecessors = []
        for node, edges in self.graph.items():
            if s in edges:
                predecessors.append(node)
        return predecessors

    def c(self, u: str, v: str) -> float:
        """
        Get the cost of moving from state u to state v.
        
        Args:
            u: Source state.
            v: Destination state.
            
        Returns:
            The cost of the edge (u,v), or infinity if the edge doesn't exist.
        """
        if u in self.graph and v in self.graph[u]:
            return self.graph[u][v]
        return float('inf')

    def update_vertex(self, u: str) -> None:
        """
        Update the rhs value of a vertex and its position in the priority queue.
        
        Args:
            u: The vertex to update.
        """
        if u != self.s_goal:
            min_cost = float('inf')
            for s_prime in self.get_successors(u):
                cost = self.c(u, s_prime) + self.g[s_prime]
                if cost < min_cost:
                    min_cost = cost
            self.rhs[u] = min_cost
        
        for i, (_, _, s) in enumerate(self.U):
            if s == u:
                self.U.pop(i)
                heapq.heapify(self.U)
                break
        
        if self.g[u] != self.rhs[u]:
            self.insert(u, self.calculate_key(u))

    def compute_shortest_path(self) -> None:
        """
        Compute the shortest path from s_start to s_goal.
        
        This is the main process of the D* Lite algorithm that updates g-values
        based on rhs-values and keeps the priority queue consistent.
        """
        while (self.top_key() < self.calculate_key(self.s_start) or 
               self.rhs[self.s_start] != self.g[self.s_start]):
            
            u = self.remove_top()
            k_old = self.top_key()
            k_new = self.calculate_key(u)
            
            if k_old < k_new:
                self.insert(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self.get_predecessors(u):
                    self.update_vertex(p)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                predecessors_and_u = self.get_predecessors(u) + [u]
                for p in predecessors_and_u:
                    self.update_vertex(p)

    def main_loop(self) -> list:
        """
        Main loop of the D* Lite algorithm.
        
        This method computes the initial plan and would handle replanning in
        a dynamic environment when edge costs change. For simplicity, this
        implementation assumes a static environment and just returns the path.
        
        Returns:
            The computed path from s_start to s_goal, or empty list if no path exists.
        """
        self.s_last = self.s_start
        self.compute_shortest_path()
        
        path = [self.s_start]
        current = self.s_start
        
        while current != self.s_goal:
            if self.g[current] == float('inf'):
                return []
            
            min_cost = float('inf')
            next_node = None
            for successor in self.get_successors(current):
                cost = self.c(current, successor) + self.g[successor]
                if cost < min_cost:
                    min_cost = cost
                    next_node = successor
            
            if next_node is None:
                return []
                
            current = next_node
            path.append(current)
        
        return path

    def get_path(self) -> list:
        """
        Compute and return the path from s_start to s_goal.
        
        Returns:
            A list of states representing the path, or empty list if no path exists.
        """
        return self.main_loop()

    def get_instruction_sequence(self) -> list:
        nodes = self.get_path()

        dirs = dict()

        with open("dirs.txt", "r") as f:
            for l in f.readlines():
                x = l.strip().split(" ")
                dirs[(int(x[0]), int(x[1]))] = x[2]

        instructions = []

        for v in zip(nodes[:-1], nodes[1:]):
            x = (int(v[0][1:]), int(v[1][1:]))
            instructions.append("F")
            instructions.append(dirs[x])

        instructions.append("F")

        return instructions


if __name__ == "__main__":
    graph = create_graph("map.txt")
    print("Graph:")
    for k, v in graph.items():
        print(f"{k}: {v}")

    def sample_path(source: str, sink: str) -> None:
        d_star = DStarLite(source, sink, graph)
        d_star.compute_shortest_path()
        print(f"Path from {source} to {sink}: {d_star.get_path()}")
        print(f"Instruction sequence from {source} to {sink}: {d_star.get_instruction_sequence()}")

    sample_path("v1", "v10")
    sample_path("v2", "v8")
    sample_path("v3", "v10")
