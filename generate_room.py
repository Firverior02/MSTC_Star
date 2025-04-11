
import os
import random

import networkx as nx

from utils.nx_graph import nx_graph_write

# Constants
ROOMS_DIR = 'data/rooms/'
NUM_ROOMS = 100
ROOM_DIMENSIONS = [10, 30, 80]
DENSITIES = [10, 20, 80, 90]


def generate_room(x: int, y: int, density: float):
    """Generate a room of dimensions x * y with density 

    Args:
        x (int): x dimension of room
        y (int): y dimension of room
        density (float): hom much of the room should be obstacles

    Raises:
        ValueError: Density must be in range [0, 1]
        ValueError: x and y must be greater than 0
    """
    
    # Argument validation
    if density < 0 or density > 1: 
        raise ValueError("Density must be in range [0,1]")
    
    if x < 1 or y < 1: 
        raise ValueError("x and y must be greater than 0")
    
    # Open/Create for room to be generated
    dir_path = f'{ROOMS_DIR}{int(density*100)}_ROOMS/{x}x{y}/'
    os.makedirs(dir_path, exist_ok=True)
    
    # Count number of files in directory
    idx = sum(1 for entry in os.listdir(dir_path) if os.path.isfile(os.path.join(dir_path, entry))) + 1

    # Room file name
    f_path = f'ROOM_{x}x{y}_{int(density*100)}_{idx}.graph'

    # Create default 2d grid of size x * y
    room = nx.grid_2d_graph(x, y)
    nodes = list(room.nodes())
    
    # How many obstacles to generate
    num_obs = round(x * y * density)

    # Place obstacles
    for _ in range(num_obs):
        # Choose a random node for the obstacle
        choice = random.choice(nodes)

        # Continue trying to place while invalid placement (graph is no longer connected)
        while _is_illegal(choice, room):
            nodes.remove(choice)
            if len(nodes) == 0:
                raise IndexError("No more nodes to assign obstacle to")
            choice = random.choice(nodes)
        
        room.remove_node(choice)
        nodes.remove(choice)
    
    # Write room to file
    nx_graph_write(room, dir_path + f_path)


    def _is_illegal(choice, room: nx.Graph):
        """Checks if an obstacle placement is invalid

        Args:
            choice (node): The node that should become an obstacle
            room (nx.Graph): The entire room graph

        Returns:
            bool: If the placement is invalid
        """
        
        # Creates a copy of the room and removed node from it
        temp = room.copy()
        temp.remove_node(choice)
        
        # If graph is no longer connected, it is an invalid placement
        return not nx.is_connected(temp)

def generate_testing_environments():
    """Generates a set of environments to test on"""
    
    # Generate rooms with different dimensions
    for dimension in ROOM_DIMENSIONS:
        # Generate rooms with different densities
        for density in DENSITIES:
            # Generate a set of rooms with these properties
            for _ in range(NUM_ROOMS):
                generate_room(dimension, dimension, density)
