from __future__ import print_function
import time
import rospy
import actionlib

from ropod_ros_msgs.msg import Action, Status
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_ros_msgs.msg import NavElevatorAction, NavElevatorGoal, NavElevatorFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class RideElevator(CommandBase):
    '''A state taking care of riding an elevator.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(RideElevator, self).__init__(name, experiment_server,
                                           outcomes=['done', 'failed'])

        self.destination_floor = kwargs.get('destination_floor', -1)
        self.timeout_s = kwargs.get('timeout_s', 120.)

        self.action_server_name = kwargs.get('elevator_action_server_name',
                                             '/ropod/take_elevator')
        self.elevator_progress_topic = kwargs.get('elevator_progress_topic',
                                                  '/ropod/take_elevator/feedback')

        self.action_server = actionlib.SimpleActionClient(self.action_server_name,
                                                                   NavElevatorAction)
        self.elevator_progress_sub = rospy.Subscriber(self.elevator_progress_topic,
                                                      NavElevatorFeedback,
                                                      self.action_progress_cb)

        # wait for a while to give the action publishers time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends a RIDE_ELEVATOR action and waits for it to finish. Returns "done" if
        the action finishes within "self.timeout_s" seconds; returns "failed" otherwise.
        '''
        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        action_goal = NavElevatorGoal()
        action_goal.action.type = 'RIDE_ELEVATOR'
        action_goal.action.goal_floor = self.destination_floor

        print('[{0}] Riding elevator to floor {1}'.format(self.name, self.destination_floor))

        self.action_server.send_goal(action_goal)
        self.wait_for_action_result(feedback_msg)

        # if the RIDE_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.action_server.cancel_all_goals()
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator ride preempted'.format(self.name))
        elif not self.action_completed or self.action_failed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator ride did not finish within the alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.elevator_progress_sub.unregister()
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes an elevator action progress message and modifies the values of
        self.action_completed or self.action_failed depending on the message status code.
        '''
        if progress_msg.feedback.feedback.status.status_code == Status.GOAL_REACHED:
            self.action_completed = True
        elif progress_msg.feedback.feedback.action_type == 'RIDE_ELEVATOR' and \
             (progress_msg.feedback.feedback.status.status_code == Status.ELEVATOR_RIDE_TIMEOUT or \
              progress_msg.feedback.feedback.status.status_code == Status.ELEVATOR_RIDE_MAP_CHANGE_FAILURE):
            self.action_failed = True

    def __report_failure(self, feedback_msg, error_str):
        '''Publishes a command feedbak message and sends a state info message.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.ExecuteExperimentFeedback -- a feedback message prefilled
                      with the command name and state
        error_str: str -- an error string to be printed on screen

        '''
        rospy.logerr('[{0}] {1}'.format(self.name, error_str))
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FAILED
        self.send_feedback(feedback_msg)
