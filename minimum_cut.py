from collections import namedtuple, deque
import numpy as np

Graph = namedtuple("Graph", "V E")

def bfs(graph, start, end, parent):
    visited = [False] * len(graph.V)
    queue = deque()
    queue.append(start)
    visited[start] = True

    while queue:
        u = queue.popleft()
        for v, weight in graph.E[u]:
            if not visited[v] and weight > 0:
                queue.append(v)
                visited[v] = True
                parent[v] = u
                if v == end:
                    return True
    return False

def mincut(graph, source, sink):
    max_flow = 0
    parent = [-1] * len(graph.V)
    
    while bfs(graph, source, sink, parent):
        path_flow = float("inf")
        s = sink
        while s != source:
            u = parent[s]
            for v, weight in graph.E[u]:
                if v == s:
                    path_flow = min(path_flow, weight)
                    break
            s = u

        max_flow += path_flow

        v = sink
        while v != source:
            u = parent[v]
            for i, (vertex, weight) in enumerate(graph.E[u]):
                if vertex == v:
                    graph.E[u][i] = (vertex, weight - path_flow)
                    break
            found = False
            for i, (vertex, weight) in enumerate(graph.E[v]):
                if vertex == u:
                    graph.E[v][i] = (vertex, weight + path_flow)
                    found = True
                    break
            if not found:
                graph.E[v].append((u, path_flow))

            v = u

        parent = [-1] * len(graph.V)
    
    A = [0]
    A[1:] = [i for i in range(len(parent)) if parent[i] != -1]
    B = [node for node in graph.V if node not in A]
    
    cut_edges = []
    for i in A:
        for neighbor, weight in graph.E[i]:
            if neighbor in B:
                cut_edges.append((i, neighbor))
    
    return max_flow, cut_edges

# Example usage
G = Graph(
    [0, 1, 2, 3],
    [
        [(1, 10), (2, 3)],  # 0
        [(3, 3), (2, 10)],  # 1
        [(3, 10)],  # 2
        []  # 3
    ]
)

max_flow, cut_edges = mincut(G, 0, 3)
print("Maximum Flow:", max_flow)
print("Edges to be deleted for minimum s-t cut:", cut_edges)