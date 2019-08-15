from __future__ import print_function
import time
import rospy
import uuid

import actionlib
from ropod_ros_msgs.msg import DockAction, DockGoal, DockFeedback
from ropod_ros_msgs.msg import Action, TaskProgressDOCK, Status
from ropod_ros_msgs.msg import Area, SubArea
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class Dock(CommandBase):
    '''
    An interface for performing docking

    @author Santosh Thoduka
    @maintainer Santosh Thoduka
    @contact santosh.thoduka@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(Dock, self).__init__(name, experiment_server, outcomes=['done', 'failed'])

        self.sub_area_id = kwargs.get('sub_area_id', '56')
        self.sub_area_name = kwargs.get('sub_area_name', 'BRSU_C_L0_C11_LA1')
        self.area_id = kwargs.get('area_id', '2')
        self.area_name = kwargs.get('area_name', 'BRSU_C_L0_C11')
        self.dock_action_server_name = kwargs.get('dock_action_server_name', '/collect_cart')
        self.dock_progress_topic = kwargs.get('dock_progress_topic', '/collect_cart/feedback')
        self.timeout_s = kwargs.get('timeout_s', 240.)
        self.action_server = actionlib.SimpleActionClient(self.dock_action_server_name, DockAction)
        self.dock_progress_sub = rospy.Subscriber(self.dock_progress_topic,
                                                   TaskProgressDOCK,
                                                   self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publisher time to initialise
        rospy.sleep(1.0)

    def execute(self, userdata):
        '''
        Sends a dock goal to the specified sub area
        '''
        self.area_list = []
        self.current_area_idx = 0

        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        action_msg = Action()
        action_msg.type = 'DOCK'
        action_msg.action_id = str(uuid.uuid4())
        area_msg = Area()
        area_msg.id = self.area_id
        area_msg.name = self.area_name
        sub_area = SubArea()
        sub_area.id = self.sub_area_id
        sub_area.name = self.sub_area_name
        area_msg.sub_areas.append(sub_area)
        action_msg.areas.append(area_msg)
        action_msg.sub_areas.append(SubArea())

        action_goal = DockGoal()
        action_goal.action = action_msg

        print('[{0}] Docking in area {1}, sub_area {2}'.format(self.name, self.area_id, self.sub_area_id))
        self.action_server.send_goal(action_goal)
        self.wait_for_action_result(feedback_msg)

        # if the GOTO action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if self.experiment_server.is_preempt_requested():
            self.action_server.cancel_all_goals()
            self.__report_failure(feedback_msg,
                                  '[{0}] dock preempted'.format(self.name))
        elif not self.action_completed or self.action_failed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Docking did not finish in time; giving up'.format(self.name))
            self.cleanup(ExecuteExperimentFeedback.FAILED)
            return 'failed'

        self.cleanup(ExecuteExperimentFeedback.FINISHED)
        return 'done'



    def action_progress_cb(self, progress_msg):
        '''Processes a dock action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if (progress_msg.feedback.feedback.status.module_code == Status.MOBIDIK_COLLECTION and
            progress_msg.feedback.feedback.status.status_code == Status.DOCKING_SEQUENCE_SUCCEEDED):
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
        self.dock_progress_sub.unregister()

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
