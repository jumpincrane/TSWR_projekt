from statemachine import StateMachine, State, Transition

# Supervisor process
# IDLE
# A = waiting for empty pallet to arrive
# B = palletizing process
# C = waiting for full pallet to exit

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

supervisor_options = [
    {"name": "IDLE", "initial": True, "value": "idle"},
    {"name": "A", "initial": False, "value": "a"},
    {"name": "B", "initial": False, "value": "b"},
    {"name": "C", "initial": False, "value": "c"}
]

sub_options = [
    {"name": "IDLE", "initial": True, "value": "idle"},
    {"name": "A", "initial": False, "value": "a"},
    {"name": "B", "initial": False, "value": "b"},
    {"name": "C", "initial": False, "value": "c"}
]

pallet_p_options = [
    {"name": "IDLE", "initial": True, "value": "idle"},
    {"name": "A", "initial": False, "value": "a"},
    {"name": "B", "initial": False, "value": "b"},
    {"name": "C", "initial": False, "value": "c"},
    {"name": "D", "initial": False, "value": "d"},
    {"name": "E", "initial": False, "value": "e"},
    {"name": "F", "initial": False, "value": "f"}]

# create State objects for a master
pallet_p_states = [State(**opt) for opt in pallet_p_options]
supervisor_states = [State(**opt) for opt in supervisor_options]
sub_states = [State(**opt) for opt in sub_options]

pallet_p_form_to = [
    [0, [1]],
    [1, [2]],
    [2, [3]],
    [3, [2, 4]],
    [4, [5]],
    [5, [6]],
    [6, [0, 5]]
]

supervisor_form_to = [
    [0, [1]],
    [1, [2]],
    [2, [3]],
    [3, [0]]
]

sub_form_to = [
    [0, [1]],
    [1, [2]],
    [2, [3]],
    [3, [0, 1]]
]

pallet_p_transitions = {}
supervisor_transitions = {}
sub_transitions = {}

for indices in pallet_p_form_to:
    from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple

    for to_idx in to_idx_tuple:  # iterate over destinations from a source state
        op_identifier = "m_{}_{}".format(from_idx, to_idx)  # parametrize identifier of a transition
        transition = Transition(pallet_p_states[from_idx], pallet_p_states[to_idx], identifier=op_identifier)
        pallet_p_transitions[op_identifier] = transition
        pallet_p_states[from_idx].transitions.append(transition)

for indices in supervisor_form_to:
    from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple
    for to_idx in to_idx_tuple:  # iterate over destinations from a source state
        op_identifier = "m_{}_{}".format(from_idx, to_idx)  # parametrize identifier of a transition
        transition = Transition(supervisor_states[from_idx], supervisor_states[to_idx], identifier=op_identifier)
        supervisor_transitions[op_identifier] = transition
        supervisor_states[from_idx].transitions.append(transition)

for indices in sub_form_to:
    from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple
    for to_idx in to_idx_tuple:  # iterate over destinations from a source state
        op_identifier = "m_{}_{}".format(from_idx, to_idx)  # parametrize identifier of a transition
        transition = Transition(sub_states[from_idx], sub_states[to_idx], identifier=op_identifier)
        sub_transitions[op_identifier] = transition
        sub_states[from_idx].transitions.append(transition)


class Gen(StateMachine):
    states = []
    transitions = []
    states_map = {}
    current_state = None

    def __init__(self, states, transitions):

        self.states = []
        self.transitions = []
        self.states_map = {}
        self.current_state = states[0]

        for s in states:
            setattr(self, str(s.name).lower(), s)
            self.states.append(s)
            self.states_map[s.value] = str(s.name)

        for key in transitions:
            setattr(self, str(transitions[key].identifier).lower(), transitions[key])
            self.transitions.append(transitions[key])

        super(Gen, self).__init__()

    # represent... class func
    def __repr__(self):
        return "{}(model={!r}, state_field={!r}, current_state={!r})".format(
            type(self).__name__, self.model, self.state_field,
            self.current_state.identifier,
        )


def main():
    pallet_process = Gen(pallet_p_states, pallet_p_transitions)
    supervisor = Gen(supervisor_states, supervisor_transitions)
    sub = Gen(sub_states, sub_transitions)
    print(pallet_process)
    pallet_path = ["m_0_1", "m_1_2", "m_2_3", "m_3_2"]
    supervisor_path = ["m_0_1", "m_1_2", "m_2_3", "m_3_0"]
    pallet_paths = [pallet_path]
    supervisor_paths = [supervisor_path]



if __name__ == '__main__':
    main()
