from statemachine import Transition
import itertools
def print_transitions(state_machine):
    print(f'Available transitions for diagram : {state_machine.name}')
    for i in state_machine.transitions:
            if i.source.name == state_machine.current_state.name:
                print(i.identifier)

def check_transition(state_machines, state_machine_name, trans_nr):
    znalazło = False
    for machine in state_machines:
        if machine.name == state_machine_name:
            for transition in machine.transitions:
                    if transition.identifier == trans_nr:
                        znalazło = True
    if znalazło == True:
        correct_nr = True
    else:
        correct_nr = False
    return correct_nr


def process(state_machines, name, nr_transition):
    for machine in state_machines:
        if machine.name == name:
            for transition in machine.transitions:
                    if transition.identifier == nr_transition:
                        number = nr_transition[-3:]
                        construct = f"callable_{number}"
                        atr = getattr(machine, construct, None)
                        if atr is not None:
                            atr()
                        machine.current_state = transition.destinations[0]
                        print(f'{machine.name} current state: {machine.current_state}')


def console_interface(master, product, pallet, supervisor_transitions, sub_transitions, pallet_p_transitions, supervisor_from_to, sub_from_to, pallet_p_from_to, redraw):
    master.name = "master"
    product.name = "product"
    pallet.name = "pallet"
    pause = "---------------------------------------------------------------------------------------------------------------------"
    print(pause)
    print("AUTOMATIC PALLETIZING PROCESS :)")
    error = "Try again, probably the value you wrote down was incorect"

    state_machines = [master, product, pallet]

    while True:
        print(pause)
        redraw(curr_supervisor=master.current_state.name, curr_sub=product.current_state.name, curr_pallet=pallet.current_state.name)
        for state in state_machines:
            print_transitions(state)
        correct_name = True
        correct_nr = True
        while correct_name:
            print('First choose name of automata and press enter')
            name = input()
            if name == 'master' or name == 'product' or name == 'pallet':
                correct_name = False
        print('Now write down number of transition in format <int>_<int> and press enter') 
        nr_transition = input()
        t = 'transition_'+nr_transition
        correct_nr = check_transition(state_machines, name, t)
        while correct_nr:
            if correct_nr == True:
                process(state_machines, name, t)
                break
            else:
                correct_nr == False


def testing(master, product, pallet, supervisor_transitions, sub_transitions, pallet_p_transitions, supervisor_from_to, sub_from_to, pallet_p_from_to, redraw):
    master.name = "master"
    product.name = "product"
    pallet.name = "pallet"
    pause = "---------------------------------------------------------------------------------------------------------------------"
    print(pause)
    state_machines = [master, product, pallet]

    answers = [("Waiting", "idle", "idle"),
               ("Palletizing", "idle", "idle"),
               ("Palletizing", "Arrival", "idle"),
               ("Palletizing", "Product ready", "idle"),
               ("Palletizing", "Product ready", "Moving"),
               ("Palletizing", "Product ready", "Latch"),
               ("Palletizing", "Product ready", "Product up"),
               ("Palletizing", "Product ready", "Latch"),
               ("Palletizing", "Product ready", "Product up"),
               ("Palletizing", "Product ready", "Moving"),
               ("Palletizing", "Product ready", "Gripper open"),
               ("Palletizing", "Product ready", "Released"),
               ("Palletizing", "Product ready", "Gripper open"),
               ("Palletizing", "Product ready", "Released"),
               ("Palletizing", "Product ready", "idle"),
               ("Palletizing", "Product up", "idle"),
               ("Palletizing", "Arrival", "idle"),
               ("Palletizing", "Product ready", "idle"),
               ("Palletizing", "Product up", "idle"),
               ("Palletizing", "idle", "idle"),
               ("Exit", "idle", "idle"),
               ("idle", "idle", "idle")]


    path = [("master", "transition_0_1"),
            ("master", "transition_1_2"),
            ("product", "transition_0_1"),
            ("product", "transition_1_2"),
            ("pallet", "transition_0_1"),
            ("pallet", "transition_1_2"),
            ("pallet", "transition_2_3"),
            ("pallet", "transition_3_2"),
            ("pallet", "transition_2_3"),
            ("pallet", "transition_3_4"),
            ("pallet", "transition_4_5"),
            ("pallet", "transition_5_6"),
            ("pallet", "transition_6_5"),
            ("pallet", "transition_5_6"),
            ("pallet", "transition_6_0"),
            ("product", "transition_2_3"),
            ("product", "transition_3_1"),
            ("product", "transition_1_2"),
            ("product", "transition_2_3"),
            ("product", "transition_3_0"),
            ("master", "transition_2_3"),
            ("master", "transition_3_0")]

    for step, ans in zip(path, answers):
        name, t = step
        master_state, product_state, pallet_state = ans

        process(state_machines, name, t)
        assert master.current_state.value == master_state
        assert product.current_state.value == product_state
        assert pallet.current_state.value == pallet_state
    print("Test completed successfully!")
