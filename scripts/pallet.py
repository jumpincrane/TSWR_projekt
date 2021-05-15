import networkx as nx
import matplotlib.pyplot as plt
import sys

#from console import *
from generator import Gen, setTransition, generate_states
from statemachine import StateMachine, State, Transition

import yaml
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

# load states for a master (way of passing args to class)
with open("../config/options.yaml", "r") as file:
    loaded = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_states = generate_states(loaded["pallet_p_options"])
supervisor_states = generate_states(loaded["supervisor_options"])
sub_states = generate_states(loaded["sub_options"])


with open("../config/from_tos.yaml", 'r') as file:
    from_to = yaml.load(file, Loader=yaml.FullLoader)

pallet_p_from_to = from_to["pallet_p_from_to"]
supervisor_from_to = from_to["supervisor_from_to"]
sub_from_to = from_to["sub_from_to"]

pallet_p_states, pallet_p_transitions = setTransition(pallet_p_from_to, pallet_p_states)
supervisor_states, supervisor_transitions = setTransition(supervisor_from_to,supervisor_states)
sub_states, sub_transitions = setTransition(from_to["sub_from_to"], sub_states)


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

    print(pallet)

    print("AUTOMATIC PALLETIZING PROCESS :)")
    print('HOW TO USE :')
    print(
        'If you want to chose a transition write down it in format <current state [int]>_<state you want to choose[int]>')
    print('To go to futher state write down value of chosen transition, to end the process write "end"')
    error = "Try again, probably the value you wrote down was incorect"

    while True:

        print(f'You are in main diagram Initial State: {master.current_state.name}')
        print("Active transitions : ")
        for i in supervisor_from_to[0][1]:
            transition_name = supervisor_transitions[f'transition_0_{i}'].identifier
            if transition_name == "transition_0_1":
                print("*    ", transition_name, ": START - motor_pallet: on")

        a = input("Your choice: ")
        if a == str('end'):
            break
        elif a == str("0_1"):
            master.current_state.name = "Waiting for empty pallet to arrive"
            print(f'You are in Main diagram state : {master.current_state.name}')

            print("Active transitions : ")
            for i in supervisor_from_to[1][1]:
                transition_name = supervisor_transitions[f'transition_1_{i}'].identifier
                if transition_name == "transition_1_2":
                    print("*    ", transition_name, ": Pallet arrived")
            b = input("Your choice: ")
            if b == str('end'):
                break
            elif b == str('1_2'):
                master.current_state.name = "Palletizing process"
                print(f'You are in Main diagram state : {master.current_state.name}')
                print(
                    "The palletizing process starts the first subordinate diagram, which is responsible for products flow")

                print(f"You are in 1-st sub diagram state : ", product.current_state.name)

                print("Active transitions : ")
                for i in sub_from_to[0][1]:
                    transition_name = sub_transitions[f'transition_0_{i}'].identifier
                    if transition_name == "transition_0_1":
                        print("*    ", transition_name, ": Start of the process : motor_object: on")
                a1 = input("Your choice: ")
                if a1 == str('end'):
                    break
                elif a1 == str('0_1'):

                    while True:
                        product.current_state.name = "Waiting for product to arrive"
                        print(f"You are in 1-st sub diagram state : ", product.current_state.name)

                        print("Active transitions : ")
                        for i in sub_from_to[1][1]:
                            transition_name = sub_transitions[f'transition_1_{i}'].identifier
                            if transition_name == "transition_1_2":
                                print("*    ", transition_name, ": Sensor detects the product : motor_object: off")
                        b1 = input("Your choice: ")
                        if b1 == str('end'):
                            break
                        elif b1 == str('1_2'):
                            product.current_state.name = "Product is ready to hold"
                            print(f"You are in 1-st sub diagram state : ", product.current_state.name)

                            while True:
                                print(
                                    "When product is ready the second subordinate diagram is on, which is responsible for robot's movemoent")

                                print(f"You are in 2-nd sub diagram state : ", pallet.current_state.name)

                                print("Active transitions : ")
                                for i in pallet_p_from_to[0][1]:
                                    transition_name = pallet_p_transitions[f'transition_0_{i}'].identifier
                                    if transition_name == "transition_0_1":
                                        print("*    ", transition_name, ": Start of product movement process")
                                a2 = input("Your choice: ")
                                if a2 == str('end'):
                                    break
                                elif a2 == str('0_1'):
                                    pallet.current_state.name = "Robot's movement towards product"
                                    print(f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                    print("Active transitions : ")
                                    for i in pallet_p_from_to[1][1]:
                                        transition_name = pallet_p_transitions[f'transition_1_{i}'].identifier
                                        if transition_name == "transition_1_2":
                                            print("*    ", transition_name, ": Goal position is aimed")
                                    b2 = input("Your choice: ")
                                    if b2 == str('end'):
                                        break
                                    elif b2 == str('1_2'):

                                        while True:
                                            pallet.current_state.name = "Gripper latches"
                                            print(f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                            print("Active transitions : ")
                                            for i in pallet_p_from_to[2][1]:
                                                transition_name = pallet_p_transitions[f'transition_2_{i}'].identifier
                                                if transition_name == "transition_2_3":
                                                    print("*    ", transition_name, ": Holding up the product")
                                            c2 = input("Your choice: ")
                                            if c2 == str('end'):
                                                break
                                            elif c2 == str('2_3'):
                                                pallet.current_state.name = "Product is up"
                                                print(
                                                    f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                                print("Active transitions : ")
                                                for i in pallet_p_from_to[3][1]:
                                                    transition_name = pallet_p_transitions[
                                                        f'transition_3_{i}'].identifier
                                                    if transition_name == "transition_3_4":
                                                        print("*    ", transition_name,
                                                              ": Gripper's sensor detects the product")
                                                    elif transition_name == "transition_3_2":
                                                        print("*    ", transition_name,
                                                              ": Gripper's sensor doesn't detect the product")
                                                d2 = input("Your choice: ")
                                                if d2 == str('end'):
                                                    break
                                                elif d2 == str('3_2'):
                                                    continue
                                                elif d2 == str('3_4'):
                                                    break
                                                else:
                                                    print(error)
                                            else:
                                                print(error)

                                        pallet.current_state.name = "Robot's movement towards pallet"
                                        print(f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                        print("Active transitions : ")
                                        for i in pallet_p_from_to[4][1]:
                                            transition_name = pallet_p_transitions[f'transition_4_{i}'].identifier
                                            if transition_name == "transition_4_5":
                                                print("*    ", transition_name, ": Goal position is aimed")
                                        e2 = input("Your choice: ")
                                        if e2 == str('end'):
                                            break
                                        elif e2 == str('4_5'):

                                            while True:
                                                pallet.current_state.name = "Gripper opens"
                                                print(
                                                    f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                                print("Active transitions : ")
                                                for i in pallet_p_from_to[5][1]:
                                                    transition_name = pallet_p_transitions[
                                                        f'transition_5_{i}'].identifier
                                                    if transition_name == "transition_5_6":
                                                        print("*    ", transition_name, ": Releasing the product")
                                                f2 = input("Your choice: ")
                                                if f2 == str('end'):
                                                    break
                                                elif f2 == str('5_6'):
                                                    pallet.current_state.name = "Product is released"
                                                    print(
                                                        f'You are in 2-st sub diagram state : {pallet.current_state.name}')

                                                    print("Active transitions : ")
                                                    for i in pallet_p_from_to[6][1]:
                                                        transition_name = pallet_p_transitions[
                                                            f'transition_6_{i}'].identifier
                                                        if transition_name == "transition_6_5":
                                                            print("*    ", transition_name,
                                                                  ": Gripper's sensor is still detecting the product")
                                                        elif transition_name == "transition_6_0":
                                                            print("*    ", transition_name,
                                                                  ": Gripper's sensor isn't detecting the product")
                                                    g2 = input("Your choice: ")
                                                    if g2 == str('end'):
                                                        break
                                                    elif g2 == str('6_5'):
                                                        continue
                                                    elif g2 == str('6_0'):
                                                        pallet.current_state.name = "Waiting for ready product to hold"
                                                        break
                                                    else:
                                                        print(error)
                                                else:
                                                    print(error)

                                        else:
                                            print(error)

                                    else:
                                        print(error)
                                else:
                                    print(error)

                            print(f"You are in 1-st sub diagram state : ", product.current_state.name)

                            print("Active transitions : ")
                            for i in sub_from_to[2][1]:
                                transition_name = sub_transitions[f'transition_2_{i}'].identifier
                                if transition_name == "transition_2_3":
                                    print("*    ", transition_name, ": Robot is holding the product")
                            c1 = input("Your choice: ")
                            if c1 == str('end'):
                                break
                            elif c1 == str('2_3'):
                                product.current_state.name = "Product is up"
                                print(f"You are in 1-st sub diagram state : ", product.current_state.name)

                                print("Active transitions : ")
                                for i in sub_from_to[3][1]:
                                    transition_name = sub_transitions[f'transition_3_{i}'].identifier
                                    if transition_name == "transition_3_0":
                                        print("*    ", transition_name, ": Pallet is full")
                                    if transition_name == "transition_3_1":
                                        print("*    ", transition_name, ": The pallet is not full")
                                d1 = input("Your choice: ")
                                if d1 == str('end'):
                                    break
                                elif d1 == str('3_0'):
                                    product.current_state.name = "Waiting for palletization process to start"
                                    break
                                elif d1 == str('3_1'):
                                    product.current_state.name = "Waiting for product to arrive"
                                    continue
                                else:
                                    print(error)
                            else:
                                print(error)
                        else:
                            print(error)

                    print(master.current_state.name)
                    print("Active transitions : ")
                    for i in supervisor_from_to[2][1]:
                        transition_name = supervisor_transitions[f'transition_2_{i}'].identifier
                        if transition_name == "transition_2_3":
                            print("*    ", transition_name, ": Process finished : The pallet is full")
                    c = input("Your choice: ")
                    if c == str('end'):
                        break
                    elif c == str('2_3'):
                        master.current_state.name = "Waiting for full pallet to exit"
                        print(f'You are in Main diagram state : {master.current_state.name}')

                        print("Active transitions : ")
                        for i in supervisor_from_to[3][1]:
                            transition_name = supervisor_transitions[f'transition_3_{i}'].identifier
                            if transition_name == "transition_3_0":
                                print("*    ", transition_name, "The pallet is full - motor_pallet : on")
                        d = input("Your choice: ")
                        if d == str('end'):
                            break
                        elif d == str('3_0'):
                            master.current_state.name = "IDLE"
                            continue
                        else:
                            print(error)
                else:
                    print(error)
            else:
                print(error)
        else:
            print(error)
    print("Exit the program")


if __name__ == '__main__':
    main()

