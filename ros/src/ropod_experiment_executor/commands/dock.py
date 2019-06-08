from __future__ import print_function
import time
import rospy
import uuid

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

        self.area_id = kwargs.get('area_id', 'MobidikArea1')
        self.area_name = kwargs.get('area_name', 'orient_wp_MobidikArea1')
        self.dock_action_topic = kwargs.get('dock_action_topic', '/ropod_task_executor/DOCK')
        self.dock_progress_topic = kwargs.get('dock_progress_topic', '/task_progress/dock')
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.dock_action_pub = rospy.Publisher(self.dock_action_topic, Action, queue_size=5)
        self.dock_progress_sub = rospy.Subscriber(self.dock_progress_topic,
                                                   TaskProgressDOCK,
                                                   self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publisher time to initialise
        rospy.sleep(1.0)

    def execute(self, userdata):
        '''Sends a navigation goal for each area in self.areas.
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
        area_msg.sub_areas.append(SubArea())
        action_msg.areas.append(area_msg)
        action_msg.sub_areas.append(SubArea())

        print('[{0}] Docking in area {1}'.format(self.name, self.area_id))
        self.dock_action_pub.publish(action_msg)
        self.action_completed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and not self.action_completed:
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)

        if not self.action_completed: # timeout occurred
            self.cleanup(ExecuteExperimentFeedback.FAILED)
            return 'failed'
        self.cleanup(ExecuteExperimentFeedback.FINISHED)
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes a dock action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if (progress_msg.status.module_code == Status.MOBIDIK_COLLECTION and
            progress_msg.status.status_code == Status.DOCKING_SEQUENCE_SUCCEEDED):
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
