import time

import matplotlib.pyplot as plt
import networkx as nx

from mcpp.mstc_star_planner import MSTCStarPlanner
from utils.nx_graph import graph_plot


class TMSTCStarPlanner(MSTCStarPlanner):
    def __init__(self, G: nx.Graph, k, R, cap, cut_off_opt=True):
        self.G = G
        self.k = k
        self.R = R
        self.capacity = cap

        self.H = self.generate_decomposed_graph(self.G, self.R)
        
        graph_plot(tmst(G), True)
        self.rho = self.generate_cover_trajectory(R[0], tmst(G))

        self.cut_off_opt = cut_off_opt
        
    def get_tree(self):
        return tmst(self.G)


def segments_share_endpoint(seg1, seg2):
    # Returns True if seg1 and seg2 share an endpoint
    return seg1[0] in seg2 or seg1[1] in seg2


def are_disjoint(edge, graph: nx.Graph):
    return nx.has_path(graph, edge[0], edge[1])
    


def tmst(G: nx.Graph, ):
    # Create bipartite graph B
    F = nx.Graph()
    H = set()
    V = set()
    flow_dict = {}

    h_index = 1
    v_index = 1

    # Step 1: Separate H and V segments
    for u, v in G.edges:
        if u[0] == v[0]:  # Horizontal segment
            node_name = f'h{h_index}'
            H.add(node_name)
            flow_dict[node_name] = (u, v)
            h_index += 1
        else:  # Vertical segment
            node_name = f'v{v_index}'
            V.add(node_name)
            flow_dict[node_name] = (u, v)
            v_index += 1

    # Step 2: Add edges between H and V if they share an endpoint
    for node in H.union(V):
        F.add_node(node)
    for h in H:
        for v in V:
            if segments_share_endpoint(flow_dict[h], flow_dict[v]):
                F.add_edge(h, v)

    # Assign bipartite sets with correct attribute name
    nx.set_node_attributes(F, {node: 0 for node in H}, name='bipartite')
    nx.set_node_attributes(F, {node: 1 for node in V}, name='bipartite')


    # Step 3: Compute maximum matching
    matching = nx.algorithms.bipartite.maximum_matching(F, top_nodes=H)
    # print("Maximum Matching:", matching)

    # Step 4: Compute minimum vertex cover using matching (König's theorem)
    minimum_vertex_cover = nx.algorithms.bipartite.to_vertex_cover(F, matching, top_nodes=H)
    # print("Minimum Vertex Cover:", minimum_vertex_cover)

    # Step 5: Compute maximum independent set as the complement
    all_nodes = set(F.nodes())
    max_independent_set = all_nodes - set(minimum_vertex_cover)
    # print("Maximum Independent Set:", max_independent_set)

    # Step 6: Visualization
    #visualize_bipartite_graph(H, V, F, minimum_vertex_cover, max_independent_set)
    
    T = nx.Graph()
    for node in G.nodes:
       T.add_node(node)
    for node in max_independent_set:
       T.add_edge(flow_dict[node][0], flow_dict[node][1], weight=G.edges[flow_dict[node]]['weight'])      
    
        
    m = nx.number_connected_components(T)

    candidate_edges = set(G.edges) - set(T.edges)

    costs = {}
    for edge in candidate_edges:
        f_i = f(edge[0], T)
        f_j = f(edge[1], T)

        temp = T.copy()
        temp.add_edge(edge[0], edge[1])
        
        fp_i = f(edge[0], temp)
        fp_j = f(edge[1], temp)

        costs[edge] = fp_i + fp_j - f_i - f_j

    # Merge bricks
    while m > 1:
        for edge in sorted(costs, key=costs.get):
            if not nx.has_path(T, edge[0], edge[1]):
                T.add_edge(edge[0], edge[1], weight=G.edges[edge]['weight'])
        m -= 1
    
    return T

def f(node, T: nx.Graph):
    if T.degree(node) == 1 or T.degree(node) == 3:
        return 2
    elif T.degree(node) == 4:
        return 4
    else:
        is_top = False
        is_bottom = False
        is_left = False
        is_right = False
        for neighbour in T.neighbors(node):
            if (neighbour[1] == node[1] + 1):
                is_top = True
            elif (neighbour[1] == node[1] - 1):
                is_bottom = True
            elif (neighbour[0] == node[0] + 1):
                is_right = True
            elif (neighbour[0] == node[0] - 1):
                is_left = True
                
        if (is_top and is_bottom) or (is_right and is_left):
            return 0
        else:
            return 2
        

def visualize_bipartite_graph(H, V, B, vertex_cover, independent_set):
    # Use bipartite_layout for clear separation
    pos = {}
    pos.update((node, (1, index)) for index, node in enumerate(H))  # Left column
    pos.update((node, (2, index)) for index, node in enumerate(V))  # Right column

    plt.figure(figsize=(10, 8))

    node_colors = []
    for node in B.nodes():
        if node in vertex_cover:
            node_colors.append('red')  # Vertex cover nodes
        elif node in independent_set:
            node_colors.append('green')  # Independent set nodes
        else:
            node_colors.append('gray')  # Unused (shouldn't happen)

    nx.draw(B, pos, with_labels=True, node_color=node_colors, node_size=700, edge_color='black')

    plt.title("Bipartite Graph\nRed: Vertex Cover, Green: Independent Set")
    plt.axis('off')
    plt.show()



