from __future__ import print_function
from abc import abstractmethod
import rospy

from ropod_ros_msgs.msg import CommandFeedback

class CommandBase(object):
    def __init__(self, name):
        self.name = name
        self._executing = False
        self._preempted = False
        self.cmd_feedback_topic = rospy.get_param('/cmd_feedback_topic', '/ropod/cmd_feedback')
        self.feedback_pub = rospy.Publisher(self.cmd_feedback_topic, CommandFeedback, queue_size=10)

    @abstractmethod
    def execute(self, params=None):
        pass

    def pause(self):
        print('[{0}] Pausing command'.format(self.name))
        self._executing = False

    def continue_execution(self):
        print('[{0}] Continuing command'.format(self.name))
        self._executing = True

    def cancel(self):
        print('[{0}] Cancelling command'.format(self.name))
        self._preempted = True
        self._executing = False

    def send_feedback(self, feedback_msg):
        self.feedback_pub.publish(feedback_msg)
