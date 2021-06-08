
def print_transitions(state_machine):
    print(f'Available transitions for diagram : {state_machine.name}')
    for i in state_machine.transitions:
            if i.source.name == state_machine.current_state.name:
                print(i.identifier)

def process(state_machines, name, nr_transition):
    for machine in state_machines:
        if machine.name == name:
            for transition in machine.transitions:
                    if transition.identifier == nr_transition:
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
        print('First choose name of automata and press enter') 
        name = input()
        print('Now write down number of transition in format <int>_<int> and press enter') 
        nr_transition = input()
        t = 'transition_'+nr_transition
        process(state_machines, name, t)