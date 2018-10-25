from __future__ import print_function
from abc import abstractmethod

class CommandBase(object):
    def __init__(self, name):
        self.name = name
        self._executing = False
        self._preempted = False

    @abstractmethod
    def execute(self, params=None):
        pass

    def pause(self):
        print('[{0}] Pausing command'.format(self.name))
        self._executing = False

    def continue_execution(self):
        print('[{0}] Continuing command'.format(self.name))
        self._executing = True

    def cancel(self):
        print('[{0}] Cancelling command'.format(self.name))
        self._preempted = True
        self._executing = False
