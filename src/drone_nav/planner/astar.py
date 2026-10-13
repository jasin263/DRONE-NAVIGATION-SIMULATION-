"""A* shortest-path finding on directed 3D navigation graph."""

import heapq
import math

import networkx as nx


def heuristic(graph: nx.DiGraph, u: str, v: str) -> float:
    """Euclidean 3D distance heuristic for A*."""
    pos_u = graph.nodes[u].get("pos")
    pos_v = graph.nodes[v].get("pos")
    if pos_u is None or pos_v is None:
        return 0.0
    return math.dist(pos_u, pos_v)


def astar_path(graph: nx.DiGraph, start: str, goal: str) -> list[str] | None:
    """Compute optimal path from start to goal node on directed graph using A*.

    Returns list of node IDs forming the route, or None if no path exists.
    """
    if start not in graph or goal not in graph:
        return None
    if start == goal:
        return [start]

    frontier: list[tuple[float, str]] = []
    heapq.heappush(frontier, (0.0, start))

    came_from: dict[str, str | None] = {start: None}
    cost_so_far: dict[str, float] = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            # Reconstruct path
            path: list[str] = []
            curr_node: str | None = goal
            while curr_node is not None:
                path.append(curr_node)
                curr_node = came_from[curr_node]
            path.reverse()
            return path

        for neighbor in graph.neighbors(current):
            edge_weight = graph[current][neighbor].get("weight", 1.0)
            new_cost = cost_so_far[current] + edge_weight

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(graph, neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    return None


def calculate_path_length(graph: nx.DiGraph, path: list[str]) -> float:
    """Calculate the total physical path distance in metres."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        weight = graph[u][v].get("weight")
        if weight is not None:
            total += weight
        else:
            total += math.dist(graph.nodes[u]["pos"], graph.nodes[v]["pos"])
    return total
