
import time

import matplotlib.pyplot as plt
import networkx as nx
from mcpp.mstc_star_planner import MSTCStarPlanner
from utils.nx_graph import graph_plot, navigate


class TMSTCStarPlanner(MSTCStarPlanner):
    def __init__(self, G: nx.Graph, k, R, cap, cut_off_opt=True):
        self.G = G  # Spanning Graph
        self.k = k  # Num robots
        self.R = R  # Depot positions for robots
        self.capacity = cap

        # Generate coverage graph
        self.H = self.generate_decomposed_graph(self.G, self.R)

        # Cache tree to avoid recomputation
        self.tree = tmst(G)

        # Generate trajectory using cached tree
        self.rho = self.generate_cover_trajectory(R[0], self.tree)

        self.cut_off_opt = cut_off_opt

    def get_tree(self):
        return self.tree


def tmst(G: nx.Graph) -> nx.Graph:
    I, H, _, flow_dict = generate_bipartite_graph(G)

    matching = nx.algorithms.bipartite.maximum_matching(I, top_nodes=H)

    minimum_vertex_cover = nx.algorithms.bipartite.to_vertex_cover(I, matching, top_nodes=H)
    max_independent_set = set(I.nodes()) - set(minimum_vertex_cover)

    T = merge_bricks(G, max_independent_set, flow_dict)

    return T


def generate_bipartite_graph(G: nx.Graph):
    I = nx.Graph()
    H = set()
    V = set()
    flow_dict = {}

    h_index = 1
    v_index = 1

    for u, v in G.edges:
        if u[0] == v[0]:  # Horizontal
            node_name = f'h{h_index}'
            H.add(node_name)
            flow_dict[node_name] = (u, v)
            h_index += 1
        else:  # Vertical
            node_name = f'v{v_index}'
            V.add(node_name)
            flow_dict[node_name] = (u, v)
            v_index += 1

    for node in H.union(V):
        I.add_node(node)

    for h in H:
        for v in V:
            if _segments_share_endpoint(flow_dict[h], flow_dict[v]):
                I.add_edge(h, v)

    return I, H, V, flow_dict


def merge_bricks(G: nx.Graph, max_independent_set, flow_dict):
    T = nx.Graph()
    for node in G.nodes:
        T.add_node(node)
    for node in max_independent_set:
        u, v = flow_dict[node]
        T.add_edge(u, v, weight=G.edges[u, v]['weight'])

    m = nx.number_connected_components(T)

    f_values = {node: _f(node, T) for node in T.nodes()}
    candidate_edges = set(G.edges) - set(T.edges)

    while m > 1:
        costs = _calc_costs(candidate_edges, T, f_values)

        for edge in sorted(costs, key=costs.get):
            u, v = edge
            if not nx.has_path(T, u, v):
                T.add_edge(u, v, weight=G.edges[u, v]['weight'])
                f_values[u] = _f(u, T)
                f_values[v] = _f(v, T)
                candidate_edges.remove(edge)
                break
            else:
                candidate_edges.remove(edge)
        m -= 1

    return T


def _calc_costs(candidate_edges, T: nx.Graph, f_values):
    costs = {}
    for u, v in candidate_edges:
        f_i = f_values[u]
        f_j = f_values[v]
        T.add_edge(u, v)
        fp_i = _f(u, T)
        fp_j = _f(v, T)
        T.remove_edge(u, v)
        costs[(u, v)] = fp_i + fp_j - f_i - f_j
    return costs


def _f(node, T: nx.Graph):
    """Number of turns around tree vertex (node)"""
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
        

def _visualize_bipartite_graph(H, V, B, vertex_cover, independent_set):
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


def _segments_share_endpoint(seg1, seg2):
    """Returns True if seg1 and seg2 share an endpoint"""
    return seg1[0] in seg2 or seg1[1] in seg2



