#!/usr/bin/python
from importlib import import_module
import smach

class StateMachine(smach.StateMachine):
    '''An interface for creating state machines from state machine description files

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, sm_data, experiment_server):
        smach.StateMachine.__init__(self, outcomes=sm_data.outcomes)

        with self:
            for _, state in sm_data.state_params.items():
                state_class = getattr(import_module(state.state_module_name),
                                      state.state_class_name)
                smach.StateMachine.add(state.name,
                                       state_class(state.name,
                                                   experiment_server,
                                                   **state.args),
                                       transitions=state.transitions)
