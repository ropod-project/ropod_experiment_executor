from __future__ import print_function
import time
import rospy
import actionlib

from ropod_ros_msgs.msg import Action, Status
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_ros_msgs.msg import NavElevatorAction, NavElevatorGoal, NavElevatorFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class EnterElevator(CommandBase):
    '''A state taking care of waiting for an elevator and entering it.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(EnterElevator, self).__init__(name, experiment_server,
                                            outcomes=['done', 'failed'])

        self.areas = kwargs.get('areas', list())
        self.area_floor = kwargs.get('area_floor', 0)
        self.elevator_id = kwargs.get('elevator_id', 1)
        self.elevator_door_id = kwargs.get('elevator_door_id', 1)
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
        '''Sends a WAIT_FOR_ELEVATOR action and waits for it to finish;
        if the elevator arrives within "self.timeout_s" seconds,
        sends an ENTER_ELEVATOR action and waits for it to finish.
        Stops the experiment if either of the actions fails.
        '''
        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        # we first send a WAIT_FOR_ELEVATOR action to the robot
        # so that it can wait for the elevator to arrive
        action_goal = NavElevatorGoal()
        action_goal.action.type = 'WAIT_FOR_ELEVATOR'
        action_goal.action.elevator.elevator_id = self.elevator_id
        action_goal.action.elevator.door_id = self.elevator_door_id
        action_goal.action.start_floor = self.area_floor
        action_goal.action.goal_floor = self.area_floor

        print('[{0}] Waiting for elevator {1} at door {2}'.format(self.name,
                                                                  self.elevator_id,
                                                                  self.elevator_door_id))
        self.action_server.send_goal(action_goal)
        self.wait_for_action_result(feedback_msg)

        # if the WAIT_FOR_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.action_server.cancel_all_goals()
            self.__report_failure(feedback_msg,
                                  '[{0}] Waiting for elevator preempted'.format(self.name))
        elif not self.action_completed or self.action_failed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator did not arrive within the alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        # if the WAIT_FOR_ELEVATOR action ends in success, we proceed
        # by sending an ENTER_ELEVATOR action to the robot
        action_goal = NavElevatorGoal()
        action_goal.action.type = 'ENTER_ELEVATOR'
        action_goal.action.start_floor = self.area_floor
        action_goal.action.goal_floor = self.area_floor

        print('[{0}] Entering elevator {1} at door {2}'.format(self.name,
                                                               self.elevator_id,
                                                               self.elevator_door_id))
        self.action_server.send_goal(action_goal)
        self.wait_for_action_result(feedback_msg)

        # if the ENTER_ELEVATOR action could not be completed successfully,
        # we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.action_server.cancel_all_goals()
            self.__report_failure(feedback_msg,
                                  '[{0}] Entering elevator preempted'.format(self.name))
        elif not self.action_completed or self.action_failed:
            self.__report_failure(feedback_msg,
                              '[{0}] Could not enter elevator; giving up'.format(self.name))
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
        elif progress_msg.feedback.feedback.action_type == 'WAIT_FOR_ELEVATOR' and \
             progress_msg.feedback.feedback.status.status_code == Status.WAITING_FOR_ELEVATOR_FAILED:
            self.action_failed = True
        elif progress_msg.feedback.feedback.action_type == 'ENTER_ELEVATOR' and \
             progress_msg.feedback.feedback.status.status_code == Status.ELEVATOR_ENTERING_FAILED:
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
