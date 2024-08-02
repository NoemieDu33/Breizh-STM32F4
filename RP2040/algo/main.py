GRAPH = {
    1:(2,),
    2:(3,4),
    3:(6,),
    4:(5,6),
    5:(7,8),
    6:(7,),
    7:(5,12),
    8:(9,10),
    9:(),
    10:(11,13),
    11:(10,12),
    12:(11,15),
    13:(14,),
    14:(13,15),
    15:(14,)
    }

visited = list()
current_pos = 1

def DFS(G, N, V):
    if N not in V:
        print(N)
        V.append(N)
        current_pos = N
        for neighbour in G[N]:
            DFS(G,neighbour,V)
            
print("DFS Algorithm:\n")
DFS(GRAPH, current_pos, visited)

def shortest_path_to_node(GRAPH, FROM, TO, path = [], BEFORE=None):
    current_node = FROM
    path.append(current_node)
    print(current_node)
    if current_node != TO:
        current_neighbours = GRAPH[current_node]
        for ngh in current_neighbours:
            if ngh!=BEFORE:
                shortest_path_to_node(GRAPH, ngh, TO, path, current_node)
    else:
        print(path)
        return path

print("SHORTEST PATH:\n")
x = shortest_path_to_node(GRAPH,13, 4)
print(x)