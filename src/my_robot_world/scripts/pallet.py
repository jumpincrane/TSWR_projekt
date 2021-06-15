#!/usr/bin/env python3

import sys
import yaml
import networkx as nx
import matplotlib.pyplot as plt
import rospy

from my_robot_world.srv import *
from console import console_interface, testing
from generator import Gen, setTransition, generate_states
from statemachine import StateMachine, State, Transition

def send_command(command):
    rospy.wait_for_service('cmd')
    try:
        send_cmd = rospy.ServiceProxy('cmd', cmd)
        send_cmd(command)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def test(self):
    # send_command(1)
    pass

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
# options_filename = "./src/my_robot_world/config/options.yaml"
# from_tos_filename = "./src/my_robot_world/config/from_tos.yaml"
# options_filename = "../config/options.yaml"
# from_tos_filename = "../config/from_tos.yaml"

from_tos_filename = sys.argv[1]
options_filename = sys.argv[2]
with open(options_filename, "r") as file:
    loaded = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_options = loaded["pallet_p_options"]
supervisor_options = loaded["supervisor_options"]
sub_options = loaded["sub_options"]

pallet_p_states = generate_states(pallet_p_options)
supervisor_states = generate_states(supervisor_options)
sub_states = generate_states(sub_options)

clbck = {"callable_0_1": test}


with open(from_tos_filename, 'r') as file:
    from_to = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_from_to = from_to["pallet_p_from_to"]
supervisor_from_to = from_to["supervisor_from_to"]
sub_from_to = from_to["sub_from_to"]

pallet_p_states, pallet_p_transitions = setTransition(pallet_p_from_to, pallet_p_states, clbck)
supervisor_states, supervisor_transitions = setTransition(supervisor_from_to,supervisor_states, clbck)
sub_states, sub_transitions = setTransition(sub_from_to, sub_states, clbck)



def create_and_show_graph(graph, states, edges, ax, color, current=""):
    plt.cla()
    # nodes are int ids, since from_to is defined in terms of 0 -> 1 etc.
    i = 0
    for dict in states:
        initial, name, value = dict.values()
        # name is node's attribute
        if name == current:
            graph.add_node(i, name=value, color="red")
        else:
            graph.add_node(i, name=value, color=color)
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
    
    pos = nx.shell_layout(graph)
    nx.draw(graph, pos, with_labels=False, ax = ax, connectionstyle='arc3, rad = 0.1', node_color=colors, alpha=0.75)

    # offset labels along y axis so they are above nodes
    pos_higher = {}
    y_off = 0.15
    for k, v in pos.items():
        pos_higher[k] = (v[0], v[1]+y_off)
    nx.draw_networkx_labels(graph, pos_higher, labels, ax=ax)

    # interactive mode so plt.show() is non-blocking
    plt.ion()
    plt.show()

fig, axs = plt.subplots(1, 3)
colors = ["green", "blue"]

def redraw(curr_pallet="", curr_supervisor="", curr_sub=""):
    create_and_show_graph(pallet_p_graph, pallet_p_options, pallet_p_from_to, axs[0], colors[0], curr_pallet)
    create_and_show_graph(supervisor_graph, supervisor_options, supervisor_from_to, axs[1], colors[0], curr_supervisor)
    create_and_show_graph(sub_states_graph, sub_options, sub_from_to,axs[2], colors[0], curr_sub)
    

for i in range(2):
    create_and_show_graph(pallet_p_graph, pallet_p_options, pallet_p_from_to, axs[0], colors[i % 2])
    create_and_show_graph(supervisor_graph, supervisor_options, supervisor_from_to, axs[1], colors[i % 2])
    create_and_show_graph(sub_states_graph, sub_options, sub_from_to,axs[2], colors[i % 2])
    plt.pause(0.1)



def main():

    # create an object
    pallet = Gen.create_object(pallet_p_states, pallet_p_transitions, pallet_p_from_to, clbck)
    product = Gen.create_object(sub_states, sub_transitions, sub_from_to, clbck)
    master = Gen.create_object(supervisor_states, supervisor_transitions, supervisor_from_to, clbck)
    # console_interface(master, product, pallet, supervisor_transitions, sub_transitions, pallet_p_transitions, supervisor_from_to, sub_from_to, pallet_p_from_to, redraw)
    testing(master, product, pallet, supervisor_transitions, sub_transitions, pallet_p_transitions, supervisor_from_to, sub_from_to, pallet_p_from_to, redraw)



if __name__ == '__main__':
    main()


