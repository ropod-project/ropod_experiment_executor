from __future__ import print_function
import time
import rospy
import actionlib

from ropod_ros_msgs.msg import Action, Area, Status
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_ros_msgs.msg import NavElevatorAction, NavElevatorGoal, NavElevatorFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class ExitElevator(CommandBase):
    '''A state taking care of exiting an elevator.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(ExitElevator, self).__init__(name, experiment_server,
                                           outcomes=['done', 'failed'])

        self.outside_area_id = kwargs.get('outside_area_id', 0)
        self.outside_area_name = kwargs.get('outside_area_name', 0)
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.elevator_action_server_name = kwargs.get('elevator_action_server_name',
                                                      '/ropod/take_elevator')
        self.elevator_progress_topic = kwargs.get('elevator_progress_topic',
                                                  '/task_progress/elevator')

        self.elevator_action_server = actionlib.SimpleActionClient(self.elevator_action_server_name,
                                                                   NavElevatorAction)
        self.elevator_progress_sub = rospy.Subscriber(self.elevator_progress_topic,
                                                      NavElevatorFeedback,
                                                      self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publishers time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends an EXIT_ELEVATOR action and waits for it to finish. Returns "done" if
        the action finishes within "self.timeout_s" seconds; returns "failed" otherwise.
        '''
        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        action_goal = NavElevatorGoal()
        action_goal.action.type = 'EXIT_ELEVATOR'

        outside_area_msg = Area()
        outside_area_msg.id = str(self.outside_area_id)
        outside_area_msg.name = self.outside_area_name
        outside_area_msg.type = 'local_area'
        action_goal.action.areas.append(outside_area_msg)

        print('[{0}] Exiting elevator; outside area ID {1}'.format(self.name, self.outside_area_id))
        self.elevator_action_server.send_goal(action_goal)
        self.__wait_for_action(feedback_msg)

        # if the EXIT_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.elevator_action_server.set_preempted()
            self.__report_failure(feedback_msg,
                                  '[{0}] Exiting elevator preempted'.format(self.name))
        elif not self.action_completed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Could not exit elevator within alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.elevator_progress_sub.unregister()
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes an elevator action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.feedback.feedback.status.status_code == Status.GOAL_REACHED:
            self.action_completed = True

    def __wait_for_action(self, feedback_msg):
        '''Waits for an action to either complete (self.action_completed
        will be set if that happens) or timeout (as specified by self.timeout_s);
        publishes periodic feedback messages about the progress as well.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.ExecuteExperimentFeedback -- a feedback message prefilled
                      with the command name and state

        '''
        self.action_completed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and \
              not self.action_completed and \
              not self.experiment_server.is_preempt_requested():
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)
        return self.action_completed

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
