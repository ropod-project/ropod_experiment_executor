from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Twist

from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class BaseMotion(CommandBase):
    '''An interface for performing base motions.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(BaseMotion, self).__init__(name, experiment_server,
                                         outcomes=['done', 'failed'])
        print('Creating state {0}'.format(name))

        self.vel_topic = kwargs.get('vel_topic', '/ropod/cmd_vel')
        self.vel_x = kwargs.get('vel_x', 0.)
        self.vel_y = kwargs.get('vel_y', 0.)
        self.vel_theta = kwargs.get('vel_theta', 0.)
        self.duration = kwargs.get('duration', 0.)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

    def execute(self, userdata):
        '''Sends a base velocity message to "self.vel_topic".
        '''
        twist_msg = Twist()

        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        elapsed = 0.
        print('[{0}] Moving with vel: x: {1} m/s, y: {2} m/s, theta: {3} rad/s'.format(self.name, self.vel_x,
                                                                                       self.vel_y, self.vel_theta))
        start_time = time.time()
        while elapsed < self.duration:
            twist_msg.linear.x = self.vel_x
            twist_msg.linear.y = self.vel_y
            twist_msg.angular.z = self.vel_theta
            self.vel_pub.publish(twist_msg)

            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)

        print('[{0}] Stopping motion'.format(self.name))
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.vel_pub.publish(twist_msg)

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FINISHED
        self.send_feedback(feedback_msg)
        return 'done'
