import time

import networkx as nx
import numpy as np
from mcpp.mstc_star_planner import MSTCStarPlanner
from mcpp.tmstc_star_planner import TMSTCStarPlanner
from utils.nx_graph import (calc_overlapping_ratio, graph_plot, mst,
                            nx_graph_read, show_result, simulation)


def test(name, G: nx.Graph, R, obs_graph):
    print(f"\n==== Running {name}====")

    if (name == 'MSTC-Star'):
        planner = MSTCStarPlanner(G, len(R), R, float('inf'), True)
    elif (name == 'TMSTC-Star'):
        planner = TMSTCStarPlanner(G, len(R), R, float('inf'), True)
    else:
        print(f'Invalid algorithm name: {name}\nExiting...')
        exit(0)
    
    plans = planner.allocate()
    paths, weights = planner.simulate(plans)
    
    paths_turns = []
    for i, path in enumerate(paths):
        turn_path = [(R[i][0],R[i][1] - 1)]
        for node in path:
            turn_path.append(node)

        turn_path.append((R[i][0],R[i][1] - 1))
        
        path_turns = {
            0: 0,
            45: 0,
            90: 0,
            135: 0,
            180: 0
        }
        
        for index in range(0, len(turn_path)-2):
            a = np.array(turn_path[index])
            b = np.array(turn_path[index + 1])
            c = np.array(turn_path[index + 2])
            
            ba = a - b
            bc = c - b
            cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
            angle = np.pi - np.arccos(cosine_angle)
            degrees = round(np.degrees(angle))
            path_turns[degrees] += 1
        
        paths_turns[i] = path_turns
            

        
    simulation(planner, paths, weights, name, 0.03, obs_graph, False, True)

    show_result(planner.get_tree(), paths, len(R))
    simulation(
        planner, paths, weights, 'Grid Map #1 - MSTC-NB', 0.03,
        obs_graph, False, False)
    print(f'{name} overlapping ratio: {calc_overlapping_ratio(paths, planner.rho)}')
    print('\n')


#prefix = 'GRID_10x10_WEIGHTED'
#prefix = 'GRID_20x20_UNWEIGHTED_FREE'
prefix = 'example1'
tmstc_star_report = 'tmstc_report4x3'
#R = [(1, 0), (2, 0), (3, 0), (4, 0)]
#R = [(0,0), (1,0), (2,0), (3,0)]
R = [(0,0), (1,0)]
G, x, y = nx_graph_read(f'data/nx_graph/{prefix}.graph')
obs_graph = nx.grid_2d_graph(int (x), int (y))
for node in G.nodes():
    obs_graph.remove_node(node)

# Run MSTC-Star
test('MSTC-Star', G, R, obs_graph)

# Run TMSTC-Star
test('TMSTC-Star', G, R, obs_graph)





