# Run from the command line with 'four_team_communication_100' as the argument
# or other text file graph. I used the group generator for creating test scenarios.

# from graph import Graph
import sys


def connectedComponents(graph):
    visited = {node: False for node in graph['nodes']}

    # DFS function to traverse the graph and mark connected components
    def dfs(vert):
        visited[vert] = True

        for neighbour in graph[vert]:
            if not visited[neighbour]:
                dfs(neighbour)

    components = 0

    # Iterates through the nodes and performs dfs on unvisited ones
    for node in graph['nodes']:
        if not visited[node]:
            dfs(node)
            components += 1

    return components


# Function to calculate edge betweenness centrality in the graph
def edgeBetweenness(graph):
    # Initialize betweenness dictionary for tracking edge betweenness values
    betweenness = {edge: 0 for edge in graph['edges']}
    betweenness.update({(v, u): 0 for (u, v) in betweenness.keys()})

    # Iterate through nodes to calculate node betweenness using BFS
    for node in graph['nodes']:
        stack, paths, dist = ([], {v: [] for v in graph['nodes']},
                              {v: float('inf') for v in graph['nodes']})
        stack.append(node)
        paths[node] = [[node]]
        dist[node] = 0

        # BFS to find the shortest paths and update betweenness values
        while stack:
            vertex = stack.pop(0)
            for neighbour in graph[vertex]:

                # if distance is inf, neighbour has not been visited
                if dist[neighbour] == float('inf'):
                    stack.append(neighbour)
                    dist[neighbour] = dist[vertex] + 1

                if dist[neighbour] == dist[vertex] + 1:
                    paths[neighbour].extend([path + [neighbour] for path in paths[vertex]])

        nodeBetweenness = {edge: 0 for edge in betweenness.keys()}

        # Update betweenness values based on the shortest paths
        for path in [path for paths in paths.values() for path in paths]:

            for edge in zip(path[:-1], path[1:]):
                nodeBetweenness[edge] += 1

        for edge in betweenness.keys():
            betweenness[edge] += nodeBetweenness[edge] / 2

    return betweenness


def remove_edge(graph, edge):
    # Remove edges both ways and from total edges

    if edge in graph['edges']:

        if edge[1] in graph[edge[0]]:
            graph[edge[0]].remove(edge[1])

        if edge[0] in graph[edge[1]]:
            graph[edge[1]].remove(edge[0])

    reverse_edge = (edge[1], edge[0])

    if reverse_edge in graph['edges']:
        graph['edges'].remove(reverse_edge)
        # print(edge)
        # print(reverse_edge)

        # Remove the original edge
        graph['edges'].remove(edge)


def print_graph(graph):
    # Find connected components (communities)
    visited = {node: False for node in graph['nodes']}
    communities = []

    def dfs(vertex, community):
        visited[vertex] = True
        community.append(vertex)
        for neighbour in graph[vertex]:
            if not visited[neighbour]:
                dfs(neighbour, community)

    for node in graph['nodes']:
        if not visited[node]:
            community = []
            dfs(node, community)
            communities.append(community)

    print("Communities:")
    for i, community in enumerate(communities, 1):
        print(f"Community {i}: {sorted(community)}")

    # print("\nRemaining edges:")
    # for edge in graph['edges']:
    # print(edge)


# Main part of the algorithm
def girvan_newman(graph):
    while connectedComponents(graph) < 4:
        # print(connectedComponents(graph))
        betweenness = edgeBetweenness(graph)
        edge_to_remove = max(betweenness, key=betweenness.get)
        # print(edge_to_remove)
        remove_edge(graph, edge_to_remove)
    return graph


def readGraph(filename):
    with open(filename, 'r') as file:
        fileLines = file.readlines()

    graph = {}
    nodes = []
    edges = []

    for line in fileLines:
        # Parse the lines and make pairs
        nodeAndNeighbours = line.rstrip().split(':')
        neighbours_str = nodeAndNeighbours[1]
        neighbours = [int(neighbor) for neighbor in neighbours_str.split(';') if neighbor.isdigit()]

        node = int(nodeAndNeighbours[0])

        # print(neighbours)

        nodes.append(node)
        graph[node] = neighbours

        # Form edge pairs
        # Edges are both ways as the graph was to be undirected
        for adjacent in neighbours:
            edge = tuple((node, adjacent))
            reverseEdge = tuple((adjacent, node))

            if edge not in edges:
                edges.append(edge)
                edges.append(reverseEdge)

    graph['nodes'] = nodes
    graph['edges'] = edges
    # print(graph['nodes'])
    # print(graph['edges'])
    # print(graph[node])

    return graph


def main():
    inGraph = sys.argv[1]
    graph = readGraph(inGraph)
    
    '''graph = {
        'nodes': list(range(21)),  # Nodes 0 to 20
        'edges': [(1, 2), (2, 3), (3, 4), (4, 5), (6, 7), (7, 8), (8, 9), (9, 10), (11, 12), (12, 13), (13, 14),
                  (14, 15), (16, 17), (17, 18), (18, 19), (19, 20)],
        0: [1],
        1: [2],
        2: [1, 3],
        3: [2, 4],
        4: [3, 5],
        5: [4],
        6: [7],
        7: [6, 8],
        8: [7, 9],
        9: [8, 10],
        10: [9],
        11: [12],
        12: [11, 13],
        13: [12, 14],
        14: [13, 15],
        15: [14],
        16: [17],
        17: [16, 18],
        18: [17, 19],
        19: [18, 20],
        20: [19],
    }'''

    # Earlier test graph
    '''graph = {
        'nodes': [0, 1, 2, 3, 4, 5, 6],
        'edges': [(0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (2, 4), (3, 4), (3, 5), (4, 5), (4, 6), (5, 6)],
        0: [1, 2],
        1: [0, 2, 3],
        2: [0, 1, 3, 4],
        3: [1, 2, 4, 5],
        4: [2, 3, 5, 6],
        5: [3, 4, 6],
        6: [4, 5]
    }'''
    result = girvan_newman(graph)
    print_graph(result)


if __name__ == "__main__":
    main()
