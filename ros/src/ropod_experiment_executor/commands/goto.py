from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, TaskProgressGOTO, Status
from ropod_ros_msgs.msg import Area
from ropod_ros_msgs.msg import CommandFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class GoTo(CommandBase):
    '''An interface for performing area navigation.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, **kwargs):
        super(GoTo, self).__init__(name, outcomes=['done', 'failed'])
        print('Creating state {0}'.format(name))

        self.areas = kwargs.get('areas', list())
        self.area_floor = kwargs.get('area_floor', 0)
        self.go_to_action_topic = kwargs.get('go_to_action_topic', '/ropod_task_executor/GOTO')
        self.go_to_progress_topic = kwargs.get('go_to_progress_topic', '/task_progress/goto')
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.go_to_action_pub = rospy.Publisher(self.go_to_action_topic, Action, queue_size=5)
        self.go_to_progress_sub = rospy.Subscriber(self.go_to_progress_topic,
                                                   TaskProgressGOTO,
                                                   self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publisher time to initialise
        rospy.sleep(0.1)

    def execute(self, userdata):
        '''Sends a navigation goal for each area in self.areas.
        '''
        action_msg = Action()
        action_msg.type = 'GOTO'
        action_msg.start_floor = self.area_floor
        action_msg.goal_floor = self.area_floor

        feedback_msg = CommandFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = CommandFeedback.ONGOING

        for area in self.areas:
            print('[{0}] Going to {1}'.format(self.name, area))

            area_msg = Area()
            area_msg.area_id = area
            area_msg.name = area
            area_msg.floor_number = self.area_floor
            action_msg.areas = [area_msg]
            self.go_to_action_pub.publish(action_msg)

            self.action_completed = False
            elapsed = 0.
            start_time = time.time()
            while elapsed < self.timeout_s and not self.action_completed:
                feedback_msg.stamp = rospy.Time.now()
                self.send_feedback(feedback_msg)
                elapsed = time.time() - start_time
                rospy.sleep(0.05)

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FINISHED
        self.send_feedback(feedback_msg)
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes a navigation action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.status.status_code == Status.COMPLETED:
            self.action_completed = True
