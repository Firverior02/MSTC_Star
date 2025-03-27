
import os
import random

import networkx as nx

from utils.nx_graph import nx_graph_write

ROOMS_DIR = 'data/rooms/'

def generate_room(x: int, y: int, density: float):
    if density < 0 or density > 1: 
        raise Exception("Density must be in range [0,1]")
    
    if x < 1 or y < 1: 
        raise Exception("x and y must be greater than 0")
    
    dir_path = f'{ROOMS_DIR}{int(density*100)}_ROOMS/{x}x{y}/'
    
    os.makedirs(dir_path, exist_ok=True)
    idx = sum(1 for entry in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, entry))) + 1

    f_path = f'ROOM_{x}x{y}_{int(density*100)}_{idx}.graph'


    room = nx.grid_2d_graph(x, y)
    num_obs = round(x * y * density)

    nodes = list(room.nodes())
    for _ in range(num_obs):
        choice = random.choice(nodes)

        while is_illegal(choice, room):
            nodes.remove(choice)
            if len(nodes) == 0:
                print('not good')
                exit(0)
            choice = random.choice(nodes)
        
        room.remove_node(choice)
        nodes.remove(choice)
    
    nx_graph_write(room, dir_path + f_path)


def is_illegal(choice, room: nx.Graph):
    temp = room.copy()
    temp.remove_node(choice)
    return not nx.is_connected(temp)


generate_room(5, 5, 0.3)
