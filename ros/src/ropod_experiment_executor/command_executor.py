from __future__ import print_function
from importlib import import_module

from experiment_executor.utils import load_yaml_file

class CommandExecutor(object):
    def __init__(self, config_file_name=None):
        self.command_names = list()
        self.command_interfaces = dict()
        if config_file_name:
            print('Initialising command executor...')
            self.init(config_file_name)
            print('Command executor initialised')

    def init(self, config_file_name):
        config_data = load_yaml_file(config_file_name)
        parent_module_name = config_data['parent_module_name']

        for command_config in config_data['commands']:
            command_name = command_config['name']

            print('Initialising interface for command {0}'.format(command_name))
            self.command_names.append(command_name)

            command_type = command_config['type']
            obj_module = '{0}.{1}'.format(parent_module_name, command_type)
            class_name = ''.join([x.title() for x in command_type.split('_')])
            CommandType = getattr(import_module(obj_module), class_name)

            self.command_interfaces[command_name] = CommandType(command_name,
                                                                **command_config['args'])

    def execute_command(self, command, params=None):
        self.command_interfaces[command].execute(params)

    def pause_command(self, command):
        self.command_interfaces[command].pause()

    def continue_command(self, command):
        self.command_interfaces[command].continue_execution()

    def cancel_command(self, command):
        self.command_interfaces[command].cancel()
