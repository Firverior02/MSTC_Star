import csv
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial

import matplotlib.animation
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from mcpp.mfc_planner import test_MFC
from mcpp.mstc_planner import test_MSTC, test_MSTC_BT_OPT
from mcpp.mstc_star_planner import (MSTCStarPlanner, test_MSTC_STAR,
                                    test_MSTC_STAR_CUT_OPT)
from mcpp.tmstc_star_planner import TMSTCStarPlanner
from utils.nx_graph import (calc_num_turns, calc_overlapping_ratio, graph_plot,
                            mst, nx_graph_read, show_result, simulation)

# Constants
ROOMS_DIR = 'data/rooms/'
NUM_ROOMS = 100
ROOM_DIMENSIONS = [10, 30, 60]
DENSITIES = [5, 15, 40, 50]
RESULTS_FNAME = 'results'

def save_results(dimension, density, algorithm, num_robots, time, overlap):
    data = [dimension, density, algorithm, num_robots, str(time).replace(".", ","), str(overlap).replace(".", ",")]

    with open(f'{RESULTS_FNAME}.csv', mode='a', newline='\n') as file:
        writer = csv.writer(file, delimiter=";")
        writer.writerow(data)
    

def test(name, G: nx.Graph, R, obs_graph, debug=False):

    if (name == 'MSTC-Star'):
        planner = MSTCStarPlanner(G, len(R), R, float('inf'), True)
    elif (name == 'TMSTC-Star'):
        planner = TMSTCStarPlanner(G, len(R), R, float('inf'), True)
    else:
        print(f'Invalid algorithm name: {name}\nExiting...')
        exit(0)
    
    plans = planner.allocate()
    paths, weights = planner.simulate(plans, False)

    if debug:
        print('Ska inte vara här!!')
        show_result(planner.get_tree(), paths, len(R))
    
    # Get final time from simulation
    time = simulation(planner, paths, weights, name, 0.03, obs_graph, False, debug)
    overlapping = calc_overlapping_ratio(paths, planner.rho)
    
    if debug:
        print(f'{name} total time: {time}s')
        print(f'{name} overlapping ratio: {overlapping}')

        paths_turns = calc_num_turns(paths, R)
        total_turns = 0
        total_degrees = 0
        for path_turns in paths_turns:
            for k, v in path_turns.items():
                total_degrees += k*v
                if not k == 0:
                    total_turns += v

        print(f'{name} number of turns: {total_turns}')
        print(f'{name} number of degrees for turns: {total_degrees}\n')

    return time, overlapping


def test_room(dimension, density, idx, robot_counts, debug=False):
    dimension_name = f'{dimension}x{dimension}'
    
    # Read G
    G, _, _ = nx_graph_read(f'data/rooms/{density}_ROOMS/{dimension_name}/ROOM_{dimension_name}_{density}_{idx}.graph')
    obs_graph = nx.grid_2d_graph(dimension, dimension)
    for node in G.nodes():
        obs_graph.remove_node(node)

    data = [] 
    for num_robots in robot_counts:
        # Select docking stations for robots
        R = []
        for node in G.nodes():
            R.append(node)
            if len(R) >= num_robots: break

        # Run MSTC-Star
        mstc_time, mstc_overlapping = test('MSTC-Star', G, R, obs_graph, debug)

        # Run TMSTC-Star
        tmstc_time, tmstc_overlapping = test('TMSTC-Star', G, R, obs_graph, debug)
        
        # # Save data
        # save_results(dimension, density, "MSTC*", num_robots, mstc_time, mstc_overlapping)
        # save_results(dimension, density, "TMSTC*", num_robots, tmstc_time, tmstc_overlapping)
        data.append([dimension, density, "MSTC*", num_robots, str(mstc_time).replace(".", ","), str(mstc_overlapping).replace(".", ",")])
        data.append([dimension, density, "TMSTC*", num_robots, str(tmstc_time).replace(".", ","), str(tmstc_overlapping).replace(".", ",")])
        
    return data

        

def test_environments(dimensions, densities, num_rooms, robot_counts, debug=False):
    """Generates a set of environments to test on"""

    
    tasks = []
    for dimension in dimensions:
        for density in densities:
            for idx in range(num_rooms):
                tasks.append((dimension, density, idx + 1, robot_counts, debug))

    with ProcessPoolExecutor() as executor:
        # Submit all tasks
        futures = [executor.submit(test_room, *task) for task in tasks]

        with open(f'{RESULTS_FNAME}.csv', mode='w', newline='\n') as file:
            writer = csv.writer(file, delimiter=";")
            writer.writerow(["Dimension", "Density", "Algorithm", "Num_Robots", "Time", "Overlapping"])
            
            for i, future in enumerate(as_completed(futures), 1):
                result = future.result()
                if result:
                    writer.writerows(result)
                if i % 10 == 0:
                    print(f"\tCompleted {i}/{len(futures)} simulations")




    # # Initialize CSV file
    # with open(f'{RESULTS_FNAME}.csv', mode='w', newline='\n') as file:
    #     writer = csv.writer(file, delimiter=";")
    #     writer.writerow(["Dimension", "Density", "Algorithm", "Num_Robots", "Time", "Overlapping"]) 

    # # Generate rooms with different dimensions
    # for dimension in dimensions:
    #     print(f"Simulating dimenison {dimension}x{dimension}")
    #     # Generate rooms with different densities
    #     for density in densities:
    #         print(f"\tSimulating density {density}%")
    #         # Generate a set of rooms with these properties
    #         for idx in range(num_rooms):
    #             if idx % 20 == 0:
    #                 print(f'\t\tSimulating index {idx}')
    #             test_room(dimension, density, idx+1, robot_counts, dimension==30)
                
                
if __name__ == '__main__':
    test_environments(ROOM_DIMENSIONS, DENSITIES, 100, [2,4], debug=False)
