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
    def __init__(self, s_start: str, s_goal: str, graph: dict):
        self.s_start = s_start
        self.s_goal = s_goal
        self.graph = graph
        self.U = []
        self.kM = 0
        self.s_last = self.s_start
        self.V = set(graph.keys())
        for neighbors in graph.values():
            self.V.update(neighbors.keys())
        self.g = {s: float('inf') for s in self.V}
        self.rhs = {s: float('inf') for s in self.V}
        self.rhs[self.s_goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.s_goal), self.s_goal))

    def calculate_key(self, s: str) -> tuple[float, float]:
        return (min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.kM, min(self.g[s], self.rhs[s]))

    def h(self, s1: str, s2: str) -> float:
        if s1 == s2:
            return 0
        if s1 in self.graph and s2 in self.graph[s1]:
            return self.graph[s1][s2]
        if s2 in self.graph:
            for neighbor, cost in self.graph[s2].items():
                if neighbor == s1:
                    return cost
        return 0

    def cost(self, u: str, v: str) -> float:
        if u in self.graph and v in self.graph[u]:
            return self.graph[u][v]
        return float('inf')

    def update_vertex(self, u: str):
        if u != self.s_goal:
            min_rhs = float('inf')
            for v in self.predecessors(u):
                min_rhs = min(min_rhs, self.g[v] + self.cost(v, u))
            self.rhs[u] = min_rhs
        self.remove_from_queue(u)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U and self.calculate_key(self.U[0][1]) < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            k_old, u = heapq.heappop(self.U)
            if k_old < self.calculate_key(u):
                heapq.heappush(self.U, (self.calculate_key(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for v in self.successors(u):
                    self.update_vertex(v)
            else:
                g_old = self.g[u]
                self.g[u] = float('inf')
                for v in self.successors(u):
                    self.update_vertex(v)
                if g_old > self.rhs[u]:
                    self.update_vertex(u)

    def predecessors(self, u: str) -> list[str]:
        preds = []
        for v in self.V:
            if v in self.graph and u in self.graph[v]:
                preds.append(v)
        return preds

    def successors(self, u: str) -> list[str]:
        if u in self.graph:
            return list(self.graph[u].keys())
        return []

    def remove_from_queue(self, u: str):
        self.U = [item for item in self.U if item[1] != u]
        heapq.heapify(self.U)

    def update_edge(self, u: str, v: str, new_cost: float):
        old_cost = self.cost(u, v)
        if old_cost > new_cost:
            if u not in self.graph:
                self.graph[u] = {}
            self.graph[u][v] = new_cost
        elif old_cost < new_cost:
            if u in self.graph and v in self.graph[u]:
                self.graph[u][v] = new_cost
            else:
                return
        self.kM += self.h(self.s_start, self.s_last)
        self.s_last = self.s_start
        self.update_vertex(u)

    def move_robot(self, new_start: str):
        self.s_start = new_start

    def get_path(self) -> list[str]:
        path = [self.s_start]
        current = self.s_start
        while current != self.s_goal:
            min_cost = float('inf')
            next_node = None
            for neighbor in self.successors(current):
                cost = self.cost(current, neighbor) + self.g[neighbor]
                if cost < min_cost:
                    min_cost = cost
                    next_node = neighbor
            if next_node:
                path.append(next_node)
                current = next_node
            else:
                break
        return path


if __name__ == "__main__":
    graph = create_graph("map.txt")
    print("Graph:")
    for k, v in graph.items():
        print(f"{k}: {v}")

    d_star = DStarLite("v1", "v10", graph)
    d_star.compute_shortest_path()
    print("Initial Path:", d_star.get_path())
