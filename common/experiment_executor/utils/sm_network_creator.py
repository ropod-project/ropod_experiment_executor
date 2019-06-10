#! /usr/bin/env python3

from os.path import join, dirname, abspath
import networkx as nx
import matplotlib.pyplot as plt

from experiment_executor.experiment_config.sm_loader import SMLoader

def create_sm_graph(sm_params, save_image_as=None):
    '''
    Create a network out of config file containing state machine information.

    :sm_params: StateMachineParams
    :save_image_as: string (file path)
    '''
    transitions = []
    for state_name, state_param in sm_params.state_params.items():
        for transition_name, target_state in state_param.transitions.items():
            transitions.append((state_name, target_state))
    G = nx.DiGraph()
    G.add_edges_from(transitions)
    width = 3
    kwargs = {
        'with_labels':True,
        'node_size':5000,    
        'node_shape':'o',# one of 'so^>v<dph8'
        'node_color':'#1f78b4',
        'edge_color':'k',
        'width':width,
        'arrowsize':width*10,
        'font_color':'k',
        'font_size':10,
    }
    nx.draw_planar(G, **kwargs)
    if save_image_as:
        plt.savefig(save_image_as)
    else:
        plt.show()

if __name__ == '__main__':
    utils_dir = abspath(dirname(__file__))
    root_dir = dirname(dirname(dirname(utils_dir)))

    config_file = join(root_dir, 'config/experiment_definitions/linear_motion.toml')
    experiment_data = SMLoader.load_sm(config_file)
    create_sm_graph(experiment_data, save_image_as=join(root_dir, 'something.png'))
