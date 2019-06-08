from __future__ import print_function
import smach

class CommandBase(smach.State):
    def __init__(self, name, experiment_server, outcomes,
                 input_keys=list(), output_keys=list()):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.sm_id = ''
        self.name = name
        self.experiment_server = experiment_server
        self.retry_count = 0
        self.executing = False
        self.succeeded = False
        self.robot_name = ''

    def execute(self, userdata):
        pass

    def send_feedback(self, feedback_msg):
        self.experiment_server.publish_feedback(feedback_msg)
