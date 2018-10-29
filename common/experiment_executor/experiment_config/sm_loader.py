from __future__ import print_function
import oyaml as yaml

from experiment_library.experiment_config.sm_params import (StateParams,
                                                            StateMachineParams,
                                                            SMFileKeys)

class SMLoader(object):
    '''An interface for loading state machine configuration files

    Author -- Alex Mitrevski
    Email -- aleksandar.mitrevski@h-brs.de

    '''
    @staticmethod
    def load_sm(sm_file):
        '''Returns an 'experiment_library.experiment_config.sm_params.StateMachineParams'
        object containing state machine description parameters

        Keyword arguments:
        sm_file -- path to a state machine file in yaml format

        '''
        sm_params = StateMachineParams()

        # we load the state machine file
        sm_data = SMLoader.__load_sm_file(sm_file)

        # we add any new arguments that are defined in the config
        # to the list of global state machine arguments
        if SMFileKeys.ARGS in sm_data:
            for arg in sm_data[SMFileKeys.ARGS]:
                arg_data = arg[SMFileKeys.ARG]
                sm_params.global_params[arg_data[SMFileKeys.ARG_NAME]] = \
                arg_data[SMFileKeys.ARG_VALUE]

        if SMFileKeys.ID in sm_data:
            sm_params.id = sm_data[SMFileKeys.ID]

        # we add all states that are defined in the config
        # to the list of state machine states
        if SMFileKeys.STATES in sm_data:
            states = sm_data[SMFileKeys.STATES]
            for state in states:
                if state not in sm_params.states:
                    sm_params.states.append(state)

        # we add any outcomes that are defined in the config
        # to the list of state machine outcomes
        if SMFileKeys.OUTCOMES in sm_data:
            outcomes = sm_data[SMFileKeys.OUTCOMES]
            for outcome in outcomes:
                if outcome not in sm_params.outcomes:
                    sm_params.outcomes.append(outcome)

        # we add any states that are defined in the config
        # to the list of state machine states
        for state_description in sm_data[SMFileKeys.STATE_DESCRIPTIONS]:
            state_data = state_description[SMFileKeys.STATE]

            state_params = StateParams()
            state_params.name = state_data[SMFileKeys.STATE_NAME]
            state_params.state_module_name = state_data[SMFileKeys.STATE_MODULE_NAME]
            state_params.state_class_name = state_data[SMFileKeys.STATE_CLASS_NAME]

            for transition in state_data[SMFileKeys.TRANSITIONS]:
                transition_data = transition[SMFileKeys.TRANSITION]
                state_params.transitions[transition_data[SMFileKeys.TRANSITION_NAME]] = \
                transition_data[SMFileKeys.RESULT_STATE]

            if SMFileKeys.ARGS in state_data:
                for arg in state_data[SMFileKeys.ARGS]:
                    arg_data = arg[SMFileKeys.ARG]
                    state_params.args[arg_data[SMFileKeys.ARG_NAME]] = \
                    arg_data[SMFileKeys.ARG_VALUE]

            for arg_name, arg_value in sm_params.global_params.items():
                arg_data = arg[SMFileKeys.ARG]
                state_params.args[arg_name] = arg_value

            # we add the state machine ID and the state name as additional state arguments
            state_params.args['sm_id'] = sm_params.id
            state_params.args['state_name'] = state_params.name

            sm_params.state_params[state_params.name] = state_params
        return sm_params

    @staticmethod
    def __load_sm_file(sm_file):
        '''Loads the file whose path is specified by 'sm_file'

        Keyword arguments:
        sm_file -- path to a state machine file in yaml format

        '''
        file_handle = open(sm_file, 'r')
        sm_data = yaml.load(file_handle)
        file_handle.close()
        return sm_data

    @staticmethod
    def __print_sm_configuration(sm_params):
        '''Prints the state machine description parameters specified by 'sm_params'

        Keyword arguments:
        sm_params -- an 'mas_execution_manager.sm_params.StateMachineParams' object

        '''
        print('State machine ID: %s' % (sm_params.id))
        print('States: %s' % (sm_params.states))
        print()
        for _, state in sm_params.state_params.items():
            print('    State: %s' % (state.name))
            print('    State module name: %s' % (state.state_module_name))
            print('    State class name: %s' % (state.state_class_name))
            print('    Transitions:')
            for transition, resulting_state in state.transitions.items():
                print('        Transition name: %s' % (transition))
                print('        Resulting state: %s' % (resulting_state))
                print()
            if state.args:
                print('    Arguments:')
                for arg, value in state.args.items():
                    print('        Name: %s' % (arg))
                    print('        Value: %s' % (value))
                    print()
