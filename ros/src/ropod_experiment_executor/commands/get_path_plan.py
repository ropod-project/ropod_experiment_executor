from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, TaskProgressGOTO, Status
from ropod_ros_msgs.msg import Area, SubArea
from ropod_ros_msgs.msg import CommandFeedback, StateInfo
from ropod_experiment_executor.commands.command_base import CommandBase

class GetPathPlan(CommandBase):
    '''An interface for loading area navigation from config files or from path planner
    depending on config.

    @author Dharmin B.
    @maintainer Alex Mitrevski

    '''
    def __init__(self, name, **kwargs):
        super(GetPathPlan, self).__init__(name, outcomes=['done', 'failed'],
                output_keys=['areas'])

        self.use_planner = kwargs.get('use_planner', True)
        if self.use_planner:
            self.timeout_s = kwargs.get('timeout_s', 20.0)
        else:
            self.area_floor = kwargs.get('area_floor', 0)
        self.areas = kwargs.get('areas', list())

    def execute(self, userdata):
        '''Sends a navigation goal for each area in self.areas.
        '''
        if self.use_planner:
            userdata.areas = self.areas
        else:
            userdata.areas = self.areas

        feedback_msg = CommandFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = CommandFeedback.ONGOING
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.SUCCESS)
        return 'done'
