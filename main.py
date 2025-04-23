import time

import matplotlib.animation
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from mcpp.mstc_star_planner import MSTCStarPlanner
from mcpp.tmstc_star_planner import TMSTCStarPlanner
from utils.nx_graph import (calc_num_turns, calc_overlapping_ratio, graph_plot,
                            mst, nx_graph_read, show_result, simulation)


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
    
    plans_list = []
    for plan in plans:
        plans_list.append(plans[plan])


    is_show = True 
    #simulation(planner, paths, weights, name, 0.03, obs_graph, False, True)

    show_result(planner.get_tree(), paths, len(R))
    simulation(
        planner, paths, weights, name, 0.03,
        obs_graph, False, is_show)
    print(f'{name} overlapping ratio: {calc_overlapping_ratio(paths, planner.rho)}')

    paths_turns = calc_num_turns(paths, R)
    total_turns = 0
    total_degrees = 0
    for path_turns in paths_turns:
        print("path_rutns: ", path_turns)
        for k, v in path_turns.items():
            total_degrees += k*v
            if not k == 0:
                total_turns += v
    print(f'{name} number of turns: {total_turns}')
    print(f'{name} number of degrees for turns: {total_degrees}')
    
    print('\n')





density = 15
dimension = 30
idx = 1
dimension_name = f"{dimension}x{dimension}"

# prefix = '30_ROOMS/5x5/ROOM_5x5_30_1'
# prefix = '20_ROOMS/20x20/ROOM_20x20_20_1'
#prefix = 'GRID_20x20_UNWEIGHTED_FREE'
#prefix = 'GRID_5x5_FREE'
prefix = 'example1'
tmstc_star_report = 'tmstc_report4x3'
#R = [(1, 0), (2, 0), (3, 0), (4, 0)]
#R = [(0,0), (1,0), (2,0), (3,0)]
R = [(0,0), (0,1)]
#R = [(0, 1), (0, 2), (0, 3)]
#G, x, y = nx_graph_read(f'data/rooms/{density}_ROOMS/{dimension_name}/ROOM_{dimension_name}_{density}_{idx}.graph')
G, x, y = nx_graph_read(f'data/nx_graph/{prefix}.graph')

obs_graph = nx.grid_2d_graph(int (x), int (y))
for node in G.nodes():
    obs_graph.remove_node(node)

# Run MSTC-Star
test('MSTC-Star', G, R, obs_graph)

# Run TMSTC-Star
test('TMSTC-Star', G, R, obs_graph)


def show(M: nx.Graph, OG: nx.Graph):
    fig = plt.figure()
    fig.set_size_inches(8, 8)
    fig.tight_layout()
    ax = plt.axes()
    # ax.margins(x=0.15, y=0.15)
    ax.axes.xaxis.set_ticklabels([])
    ax.axes.yaxis.set_ticklabels([])
    plt.grid(True)
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    for s, t in M.edges():
            x1, y1 = s
            x2, y2 = t
            ax.plot([x1, x2], [y1, y2], '-ok', mfc='r')

    for n in OG.nodes():
        ax.plot(n[0], n[1], 'xk', ms=10, mew=3)
    for s, t in OG.edges():
        x1, y1 = s
        x2, y2 = t
        ax.plot([x1, x2], [y1, y2], '-xk', ms=10, mew=3)
    plt.show()


#show(G, obs_graph)
