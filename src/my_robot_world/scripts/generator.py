from statemachine import StateMachine, State, Transition

someList = [1, 2, 3]
class Gen(StateMachine):
    states = []
    transitions = []
    states_map = {}
    current_state = None

    def __init__(self, states, transitions, form, callbacks: dict):

        # creating each new object needs clearing its variables (otherwise they're duplicated)
        self.states = []
        self.transitions = []
        self.states_map = {}
        self.current_state = states[0]

        # create fields of states and transitions using setattr()
        # create lists of states and transitions
        # create states map - needed by StateMachine to map states and its values
        for s in states:
            setattr(self, str(s.name).lower(), s)
            self.states.append(s)
            self.states_map[s.value] = str(s.name)

        for indices in form:
            from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple

            for to_idx in to_idx_tuple:  # iterate over destinations from a source state
                op_identifier = "callable_{}_{}".format(from_idx, to_idx)  # parametrize identifier of a transition
                transition_identifier = f"transition_{from_idx}_{to_idx}"
                if op_identifier in callbacks.keys():
                    callback_id = f"on_{transition_identifier}"
                    setattr(Gen, callback_id, callbacks[op_identifier])
                transition = self.states[from_idx].to(self.states[to_idx])
                transition.identifier = transition_identifier
                cb = transition.__get__(self, Gen)
                setattr(self, op_identifier, cb)

        for key in transitions:
            setattr(self, str(transitions[key].identifier).lower(), transitions[key])
            self.transitions.append(transitions[key])

        # super() - allows us to use methods of StateMachine in our Generator object
        super(Gen, self).__init__()

    # define a printable introduction of a class
    # represent... class func
    def __repr__(self):
        return "{}(model={!r}, state_field={!r}, current_state={!r})".format(
            type(self).__name__, self.model, self.state_field,
            self.current_state.identifier)

    # method of creating objects in a flexible way (we can define multiple functions
    # which will create objects in different ways)
    @classmethod
    def create_object(cls, states, transitions, form, callbacks) -> 'Gen':
        return cls(states, transitions, form, callbacks)




def setTransition(form, object_states, callbacks: dict):

    object_transitions = {}

    for indices in form:
        from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple

        for to_idx in to_idx_tuple:  # iterate over destinations from a source state
            op_identifier = "transition_{}_{}".format(from_idx, to_idx)  # parametrize identifier of a transition

            if op_identifier in callbacks.keys():
            # create transition object and add it to the master_transitions dict
                transition = Transition(object_states[from_idx], object_states[to_idx], identifier=op_identifier, on_execute=callbacks[op_identifier])
            else:
                transition = Transition(object_states[from_idx], object_states[to_idx], identifier=op_identifier)
            object_transitions[op_identifier] = transition
            # add transition to source state
            object_states[from_idx].transitions.append(transition)

    return object_states, object_transitions

def generate_states(options):
    # create State objects for a master
    # ** -> unpack dict to args
    return  [State(**opt) for opt in options]





