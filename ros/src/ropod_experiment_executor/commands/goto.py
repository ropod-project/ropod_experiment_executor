from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, TaskProgressGOTO, Status
from ropod_ros_msgs.msg import Area, SubArea
from ropod_ros_msgs.msg import CommandFeedback, StateInfo
from ropod_experiment_executor.commands.command_base import CommandBase

class GoTo(CommandBase):
    '''An interface for performing area navigation.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, **kwargs):
        super(GoTo, self).__init__(name, outcomes=['done', 'failed'],
                input_keys=['areas'])

        self.area_floor = kwargs.get('area_floor', 0)
        self.go_to_action_topic = kwargs.get('go_to_action_topic', '/ropod_task_executor/GOTO')
        self.go_to_progress_topic = kwargs.get('go_to_progress_topic', '/task_progress/goto')
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.go_to_action_pub = rospy.Publisher(self.go_to_action_topic, Action, queue_size=5)
        self.go_to_progress_sub = rospy.Subscriber(self.go_to_progress_topic,
                                                   TaskProgressGOTO,
                                                   self.action_progress_cb)
        self.action_completed = False
        self.area_list = []
        self.current_area_idx = 0

        self.waypoint_counter = 0
        self.number_of_waypoints = 0

        # wait for a while to give the action publisher time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends a navigation goal for each area in userdata.areas.
        '''
        self.area_list = []
        self.current_area_idx = 0

        feedback_msg = CommandFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = CommandFeedback.ONGOING

        action_msg = Action()
        action_msg.type = 'GOTO'
        action_msg.start_floor = self.area_floor
        action_msg.goal_floor = self.area_floor

        for area_data in userdata.areas:
            area = area_data['area_name'].encode('ascii')
            self.area_list.append(area)

            area_msg = Area()
            area_msg.id = area_data['area_id'].encode('ascii')
            area_msg.name = area_data['area_name'].encode('ascii')
            area_msg.type = area_data['area_type'].encode('ascii')
            area_msg.floor_number = self.area_floor

            subarea_msg = SubArea()
            subarea_msg.id = area_data['subarea_id'].encode('ascii')
            subarea_msg.name = area_data['subarea_name'].encode('ascii')
            subarea_msg.floor_number = self.area_floor

            area_msg.sub_areas.append(subarea_msg)
            action_msg.areas.append(area_msg)

        self.number_of_waypoints = len(userdata.areas)

        print('[{0}] Going to {1}'.format(self.name, self.area_list[self.current_area_idx]))
        self.go_to_action_pub.publish(action_msg)
        self.action_completed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and not self.action_completed:
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)

        print(self.action_completed)
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.SUCCESS)
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes a navigation action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.status.status_code == Status.REACHED and progress_msg.sequenceNumber > 0:
            self.waypoint_counter += 1
            if self.waypoint_counter == self.number_of_waypoints:
                self.action_completed = True
        elif progress_msg.status.status_code == Status.ONGOING:
            self.current_area_idx += 1
            print('[{0}] Going to {1}'.format(self.name, self.area_list[self.current_area_idx]))
            self.send_state(StateInfo.SUCCESS)
