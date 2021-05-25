import sys
import yaml
import networkx as nx
import matplotlib.pyplot as plt

from console import console_interface
from generator import Gen, setTransition, generate_states
from statemachine import StateMachine, State, Transition


# Supervisor process
# IDLE                                                  +
# A = waiting for empty pallet to arrive                +
# B = palletizing process                               +
# C = waiting for full pallet to exit                   +

# Sub process
# IDLE = waiting for palletization process to start
# A = waiting for product to arrive
# B = product is ready to hold
# C = product is up

# Palletization Process
# IDLE = waiting for ready product to hold
# A = robot's movement towards product
# B = gripper latches
# C = product is up
# D = robot's movement towards pallet
# E = gripper open
# F = product is released

pallet_p_graph = nx.MultiDiGraph()
supervisor_graph = nx.MultiDiGraph()
sub_states_graph = nx.MultiDiGraph()
# load states for a master (way of passing args to class)
with open("../config/options.yaml", "r") as file:
    loaded = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_options = loaded["pallet_p_options"]
supervisor_options = loaded["supervisor_options"]
sub_options = loaded["sub_options"]

pallet_p_states = generate_states(pallet_p_options)
supervisor_states = generate_states(supervisor_options)
sub_states = generate_states(sub_options)


with open("../config/from_tos.yaml", 'r') as file:
    from_to = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_from_to = from_to["pallet_p_from_to"]
supervisor_from_to = from_to["supervisor_from_to"]
sub_from_to = from_to["sub_from_to"]

pallet_p_states, pallet_p_transitions = setTransition(pallet_p_from_to, pallet_p_states)
supervisor_states, supervisor_transitions = setTransition(supervisor_from_to,supervisor_states)
sub_states, sub_transitions = setTransition(sub_from_to, sub_states)


def create_and_show_graph(graph, states, edges, fig):
    # nodes are int ids, since from_to is defined in terms of 0 -> 1 etc.
    i = 0
    for dict in states:
        initial, name, value = dict.values()
        # name is node's attribute
        if initial:
            graph.add_node(i, name=name, color="red")
        else:
            graph.add_node(i, name=name, color="green")

        i+=1

    # get all labels for drawing
    labels = nx.get_node_attributes(graph, 'name')
    # get colors of all the nodes
    colors = nx.get_node_attributes(graph, 'color').values()
    # add edges from list such as: [from_node, [to_node1, to_node2, ..., to_nodeN]]
    for edge in edges: 
        from_node = edge[0]
        for to_node in edge[1]:
            graph.add_edge(from_node, to_node)

    plt.figure(fig, figsize=(10.0, 10.0))
    
    pos = nx.shell_layout(graph)
    nx.draw(graph, pos, with_labels=False, connectionstyle='arc3, rad = 0.1', node_color=colors, alpha=0.75)

    # offset labels along y axis so they are above nodes
    pos_higher = {}
    y_off = 0.1  
    for k, v in pos.items():
        pos_higher[k] = (v[0], v[1]+y_off)
    nx.draw_networkx_labels(graph, pos_higher, labels)

    # scale x axis to fit labels
    l, r = plt.xlim()
    plt.xlim(l-0.3, r+0.3)

    # interactive mode so plt.show() is non-blocking
    plt.ion()
    plt.show()

create_and_show_graph(pallet_p_graph, pallet_p_options, pallet_p_from_to, 0)
create_and_show_graph(supervisor_graph, supervisor_options, supervisor_from_to, 1)
create_and_show_graph(sub_states_graph, sub_options, sub_from_to, 2)


def main():
    #pallet_process = Gen(pallet_p_states, pallet_p_transitions)
    #supervisor = Gen(supervisor_states, supervisor_transitions)
    #sub = Gen(sub_states, sub_transitions)
    #print(pallet_process)
    #pallet_path = ["m_0_1", "m_1_2", "m_2_3", "m_3_2"]
    #supervisor_path = ["m_0_1", "m_1_2", "m_2_3", "m_3_0"]
    #pallet_paths = [pallet_path]
    #supervisor_paths = [supervisor_path]

    # create an object
    pallet = Gen.create_object(pallet_p_states, pallet_p_transitions)
    product = Gen.create_object(sub_states, sub_transitions)
    master = Gen.create_object(supervisor_states, supervisor_transitions)
    console_interface(master, product, pallet, supervisor_transitions, sub_transitions, pallet_p_transitions, supervisor_from_to, sub_from_to, pallet_p_from_to)

    print(pallet)




if __name__ == '__main__':
    main()

