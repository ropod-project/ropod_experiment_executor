from __future__ import print_function
import rospy
import smach

from ropod_ros_msgs.msg import CommandFeedback, StateInfo

class CommandBase(smach.State):
    def __init__(self, name, outcomes,
                 input_keys=list(), output_keys=list()):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.sm_id = ''
        self.name = name
        self.retry_count = 0
        self.executing = False
        self.succeeded = False
        self.robot_name = ''
        self.state_info_msg = StateInfo()

        self.cmd_feedback_topic = rospy.get_param('~cmd_feedback_topic', '/ropod/cmd_feedback')
        self.state_info_topic = rospy.get_param('~state_info_topic', '/ropod/state_info')
        self.feedback_pub = rospy.Publisher(self.cmd_feedback_topic, CommandFeedback, queue_size=10)
        self.state_info_pub = rospy.Publisher(self.state_info_topic, StateInfo, queue_size=10)

    def execute(self, userdata):
        pass

    def send_feedback(self, feedback_msg):
        self.feedback_pub.publish(feedback_msg)

    def send_state(self, state):
        self.state_info_msg.state = state
        self.state_info_pub.publish(self.state_info_msg)
