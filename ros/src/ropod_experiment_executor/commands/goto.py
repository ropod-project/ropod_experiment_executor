from __future__ import print_function
import time
import rospy
import actionlib

from ropod_ros_msgs.msg import GoToAction, GoToGoal, GoToFeedback
from ropod_ros_msgs.msg import Action, Status
from ropod_ros_msgs.msg import Area, SubArea
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class GoTo(CommandBase):
    '''An interface for performing area navigation.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(GoTo, self).__init__(name, experiment_server,
                                   outcomes=['done', 'failed'],
                                   input_keys=['areas', 'area_floor'])

        self.action_server_name = kwargs.get('goto_action_server_name',
                                             '/ropod/goto')
        self.go_to_progress_topic = kwargs.get('go_to_progress_topic',
                                               '/ropod/goto/feedback')
        self.timeout_s = kwargs.get('timeout_s', 120.)

        self.action_server = actionlib.SimpleActionClient(self.action_server_name,
                                                          GoToAction)
        self.go_to_progress_sub = rospy.Subscriber(self.go_to_progress_topic,
                                                   GoToFeedback,
                                                   self.action_progress_cb)
        self.action_completed = False
        self.area_list = []

        # wait for a while to give the action publisher time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends a navigation goal for each area in userdata.areas.
        '''
        self.area_list = []

        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        if ('areas' not in userdata.keys()) or (len(userdata.areas) == 0) or \
                ('area_floor' not in userdata.keys()):
            self.cleanup(ExecuteExperimentFeedback.FINISHED)
            return 'done'

        action_goal = GoToGoal()
        action_goal.action.type = 'GOTO'
        action_goal.action.start_floor = userdata.area_floor
        action_goal.action.goal_floor = userdata.area_floor

        for area_data in userdata.areas:
            area = area_data['area_name'].encode('ascii')
            self.area_list.append(area)

            area_msg = Area()
            area_msg.id = area_data['area_id'].encode('ascii')
            area_msg.name = area_data['area_name'].encode('ascii')
            area_msg.type = area_data['area_type'].encode('ascii')
            area_msg.floor_number = userdata.area_floor

            subarea_msg = SubArea()
            subarea_msg.id = area_data['subarea_id'].encode('ascii')
            subarea_msg.name = area_data['subarea_name'].encode('ascii')
            subarea_msg.floor_number = userdata.area_floor

            area_msg.sub_areas.append(subarea_msg)
            action_goal.action.areas.append(area_msg)

        print('[{0}] Going to first area {1}'.format(self.name, self.area_list[0]))
        self.action_server.send_goal(action_goal)
        self.wait_for_action_result(feedback_msg)

        # if the GOTO action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.action_server.cancel_all_goals()
            self.__report_failure(feedback_msg,
                                  '[{0}] Waiting for elevator preempted'.format(self.name))
        elif not self.action_completed or self.action_failed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator did not arrive within the alloted time; giving up'.format(self.name))
            self.cleanup(ExecuteExperimentFeedback.FAILED)
            return 'failed'

        self.cleanup(ExecuteExperimentFeedback.FINISHED)
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes a navigation action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.feedback.feedback.status.status_code == Status.GOAL_REACHED and \
                progress_msg.feedback.feedback.sequenceNumber > 0:
            print('[{0}] Going to {1}/{2}'.format(self.name, progress_msg.feedback.feedback.sequenceNumber,
                                                  progress_msg.feedback.feedback.totalNumber))
            if progress_msg.feedback.feedback.sequenceNumber == progress_msg.feedback.feedback.totalNumber:
                self.action_completed = True

    def cleanup(self, last_feedback):
        '''Does the following cleanup
            - send the last feedback message
            - send the state status
            - unregister sub

        :last_feedback: int
        :state: int
        '''
        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = last_feedback
        self.send_feedback(feedback_msg)
        self.go_to_progress_sub.unregister()

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
