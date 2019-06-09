"""
This module implements a smach state which either loads areas from config file
or gets them from path planner depending on the config.
"""
from __future__ import print_function
import time
import rospy
from actionlib import SimpleActionClient

from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_ros_msgs.msg import GetPathPlanAction, GetPathPlanGoal
from ropod_experiment_executor.commands.command_base import CommandBase

class GetPathPlan(CommandBase):
    '''An interface for loading area navigation from config files or from path planner
    depending on config.

    @author Dharmin B.
    @maintainer Alex Mitrevski

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(GetPathPlan, self).__init__(
            name,
            experiment_server,
            outcomes=['done', 'failed'],
            input_keys=[],
            output_keys=['areas', 'area_floor'])

        self.use_planner = kwargs.get('use_planner', True)
        if self.use_planner:
            self.timeout_s = kwargs.get('timeout_s', 20.0)
            self.path_plan_action_topic = kwargs.get('path_plan_action_topic', '/get_path_plan')
            self.path_planner_client = SimpleActionClient(
                self.path_plan_action_topic,
                GetPathPlanAction)
            rospy.loginfo("Waiting to connect to path planner server")
            self.path_planner_client.wait_for_server()
            rospy.loginfo("Successfully connected to path planner server")
            self.source = kwargs.get('source', None)
            self.destination = kwargs.get('destination', None)
            self.action_completed = False
            self.result = None
        else:
            self.area_floor = kwargs.get('area_floor', 0)
            self.areas = kwargs.get('areas', list())

    def execute(self, userdata):
        '''Sends a navigation goal for each area in self.areas.
        '''
        if self.use_planner:
            if not self.source or not self.destination:
                return 'failed'

            feedback_msg = ExecuteExperimentFeedback()
            feedback_msg.command_name = self.name
            feedback_msg.state = ExecuteExperimentFeedback.ONGOING

            userdata.areas = []
            req = GetPathPlanGoal(
                start_floor=self.source['floor'],
                start_area=self.source['area_name'],
                start_sub_area=self.source['subarea_name'],
                destination_floor=self.destination['floor'],
                destination_area=self.destination['area_name'],
                destination_sub_area=self.destination['subarea_name'])
            self.path_planner_client.send_goal(req, done_cb=self.path_planner_cb)

            elapsed = 0.
            start_time = time.time()
            while elapsed < self.timeout_s and not self.action_completed:
                feedback_msg.stamp = rospy.Time.now()
                self.send_feedback(feedback_msg)
                elapsed = time.time() - start_time
                rospy.sleep(0.05)

            if not self.action_completed: # timeout occurred
                feedback_msg.stamp = rospy.Time.now()
                feedback_msg.state = ExecuteExperimentFeedback.FINISHED
                self.send_feedback(feedback_msg)
                return 'failed'

            areas = []
            for area in self.result.path_plan.areas:
                area_dict = {}
                area_dict['area_id'] = area.id
                area_dict['area_name'] = area.name
                area_dict['area_type'] = area.type
                area_dict['subarea_id'] = area.sub_areas[0].id if area.sub_areas else ""
                area_dict['subarea_name'] = area.sub_areas[0].name if area.sub_areas else ""
                areas.append(area_dict)
            userdata.areas = areas
            userdata.area_floor = self.source['floor']
        else:
            userdata.areas = self.areas
            userdata.area_floor = self.area_floor

        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FINISHED
        self.send_feedback(feedback_msg)
        return 'done'

    def path_planner_cb(self, status, result):
        """Callback to get the result from the path planner. Sets self.result if
        path planner call succeded.

        :status: int
        :result: ropod_ros_msgs/PathPlan
        :returns: None

        """
        self.action_completed = status == 3 #succeded
        self.result = result
