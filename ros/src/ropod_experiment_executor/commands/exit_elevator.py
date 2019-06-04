from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, Area, TaskProgressELEVATOR, Status
from ropod_ros_msgs.msg import CommandFeedback, StateInfo
from ropod_experiment_executor.commands.command_base import CommandBase

class ExitElevator(CommandBase):
    '''A state taking care of exiting an elevator.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, **kwargs):
        super(ExitElevator, self).__init__(name, outcomes=['done', 'failed'])

        self.outside_area_id = kwargs.get('outside_area_id', 0)
        self.outside_area_name = kwargs.get('outside_area_name', 0)
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.exit_elevator_action_topic = kwargs.get('exit_elevator_action_topic',
                                                     '/ropod_task_executor/EXIT_ELEVATOR')
        self.elevator_progress_topic = kwargs.get('elevator_progress_topic',
                                                  '/task_progress/elevator')

        self.exit_elevator_pub = rospy.Publisher(self.exit_elevator_action_topic,
                                                 Action, queue_size=5)
        self.elevator_progress_sub = rospy.Subscriber(self.elevator_progress_topic,
                                                      TaskProgressELEVATOR,
                                                      self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publishers time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends an EXIT_ELEVATOR action and waits for it to finish. Returns "done" if
        the action finishes within "self.timeout_s" seconds; returns "failed" otherwise.
        '''
        feedback_msg = CommandFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = CommandFeedback.ONGOING

        action_msg = Action()
        action_msg.type = 'EXIT_ELEVATOR'

        outside_area_msg = Area()
        outside_area_msg.id = self.outside_area_id
        outside_area_msg.name = self.outside_area_name
        outside_area_msg.type = 'local_area'
        action_msg.areas.append(outside_area_msg)

        print('[{0}] Exiting elevator; outside area ID {1}'.format(self.name, self.outside_area_id))
        self.exit_elevator_pub.publish(action_msg)
        self.__wait_for_action(feedback_msg)

        # if the EXIT_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if not self.action_completed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Could not exit elevator within alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.SUCCESS)
        self.elevator_progress_sub.unregister()
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes an elevator action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.status.status_code == Status.GOAL_REACHED:
            self.action_completed = True

    def __wait_for_action(self, feedback_msg):
        '''Waits for an action to either complete (self.action_completed
        will be set if that happens) or timeout (as specified by self.timeout_s);
        publishes periodic feedback messages about the progress as well.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.CommandFeedback -- a feedback message prefilled
                      with the command name and state

        '''
        self.action_completed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and not self.action_completed:
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)
        return self.action_completed

    def __report_failure(self, feedback_msg, error_str):
        '''Publishes a command feedbak message and sends a state info message.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.CommandFeedback -- a feedback message prefilled
                      with the command name and state
        error_str: str -- an error string to be printed on screen

        '''
        print(error_str)
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FAILED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.ERROR)
