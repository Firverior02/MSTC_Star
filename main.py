import time

import networkx as nx

from mcpp.mstc_star_planner import MSTCStarPlanner
from utils.nx_graph import nx_graph_read, simulation

#prefix = 'GRID_10x10_WEIGHTED'
prefix = 'GRID_20x20_UNWEIGHTED_FREE'
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