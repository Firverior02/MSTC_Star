import time

import networkx as nx

from mcpp.mstc_star_planner import MSTCStarPlanner
from mcpp.tmstc_star_planner import TMSTCStarPlanner
from utils.nx_graph import (calc_overlapping_ratio, graph_plot, mst,
                            nx_graph_read, show_result, simulation)

#prefix = 'GRID_10x10_WEIGHTED'
prefix = 'GRID_20x20_UNWEIGHTED_FREE'
tmstc_star_report = 'tmstc_report4x3'
#R = [(1, 0), (2, 0), (3, 0), (4, 0)]
#R = [(0,0), (1,0), (2,0), (3,0)]
R = [(0,0), (1,0)]
G = nx_graph_read(f'data/nx_graph/{prefix}.graph')
obs_graph = nx.grid_2d_graph(20, 20)
for node in G.nodes():
    obs_graph.remove_node(node)

# Run MSTC-Star
print("\n==== Running MSTC-Star ====")
planner1 = MSTCStarPlanner(G, len(R), R, float('inf'), True)
plans1 = planner1.allocate()
paths1, weights1 = planner1.simulate(plans1)
simulation(planner1, paths1, weights1, 'MSTC-Star', 0.03, obs_graph, False, True)

show_result(mst(G), paths1, len(R))
simulation(
    planner1, paths1, weights1, 'Grid Map #1 - MSTC-NB', 0.03,
    obs_graph, False, False)
print(f'MSTC overlapping ratio: {calc_overlapping_ratio(paths1, planner1.rho)}')
print('\n')


# print("\n==== Running TMSTC-Star ====")
# planner2 = TMSTCStarPlanner(G, len(R), R, float('inf'), True)
# plans2 = planner2.allocate()
# paths2, weights2 = planner2.simulate(plans2)
# simulation(planner2, paths2, weights2, 'TMSTC-Star', 0.03, obs_graph, False, True)
# print(planner2.get_tree())
