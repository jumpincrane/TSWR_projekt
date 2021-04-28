from statemachine import StateMachine, State
from statemachine.mixins import MachineMixin

class TrafficLightMachine(StateMachine):
    "A traffic light machine"
    green = State('Green', initial=True)
    yellow = State('Yellow')
    red = State('Red')

    slowdown = green.to(yellow)
    stop = yellow.to(red)
    go = red.to(green)

    cycle = green.to(yellow) | yellow.to(red) | red.to(green)

    def on_slowdown(self):
        print('Slow down!')
    def on_stop(self):
        print('Stop!')
    def on_go(self):
        print("Let's go!")

class MyModel0(object):
    def __init__(self, state):
        self.state = state

class CampaignMachineWithKeys(StateMachine):
    "A workflow machine"
    draft = State('Draft', initial=True, value=1)
    producing = State('Being produced', value=2)
    closed = State('Closed', value=3)
    cancelled = State('Cancelled', value=4)

    add_job = draft.to.itself() | producing.to.itself()
    produce = draft.to(producing)
    deliver = producing.to(closed)
    cancel = cancelled.from_(draft, producing)



class MyModel(MachineMixin):
    state_machine_name = 'CampaignMachineWithKeys'

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
        super(MyModel, self).__init__()

    def __repr__(self):
        return "{}({!r})".format(type(self).__name__, self.__dict__)

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def main():
    traffic_light = TrafficLightMachine()
    print(traffic_light.current_state)
    print(traffic_light.current_state == TrafficLightMachine.green == traffic_light.green)
    print(traffic_light.is_green)
    print(traffic_light.is_yellow)
    print(traffic_light.is_red)
    print(traffic_light.slowdown())
    print(traffic_light.current_state)
    print(traffic_light.is_yellow)
    #print(traffic_light.slowdown())
    traffic_light.run('stop')
    print(traffic_light.is_red)
    machine = TrafficLightMachine(start_value='red')
    print(traffic_light.is_red)

# making object depending on another object
    obj = MyModel0(state='red')
    traffic_light = TrafficLightMachine(obj)
    print(traffic_light.is_red)
    print(obj.state)
    obj.state = 'green'
    print(traffic_light.is_green)
    traffic_light.slowdown()
    print(obj.state)
    print(traffic_light.is_yellow)


#Callbacks
    stm = TrafficLightMachine()
    stm.slowdown()
    stm.stop()
    stm.go()
    stm.cycle()
    stm.cycle()
    stm.cycle()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
