from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Twist

from ropod_experiment_executor.commands.command_base import CommandBase

class BaseMotion(CommandBase):
    '''An interface for performing base motions.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, **kwargs):
        super(BaseMotion, self).__init__(name)

        self.vel_topic = kwargs.get('vel_topic', '/ropod/cmd_vel')
        self.vel_x = kwargs.get('vel_x', 0.)
        self.vel_y = kwargs.get('vel_y', 0.)
        self.vel_theta = kwargs.get('vel_theta', 0.)
        self.duration = kwargs.get('duration', 0.)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)

    def execute(self, params=None):
        '''Sends a base velocity message to "self.vel_topic".
        '''
        twist_msg = Twist()
        if params:
            self.vel_x = params.linear.x
            self.vel_y = params.linear.y
            self.vel_theta = params.angular.z

        elapsed = 0.
        elapsed_before_pause = 0.
        pause_signal_sent = False

        print('[{0}] Moving with vel: x: {1} m/s, y: {2} m/s, theta: {3} rad/s'.format(self.name, self.vel_x,
                                                                                       self.vel_y, self.vel_theta))
        self._executing = True
        start_time = time.time()
        while elapsed < self.duration and not self._preempted:
            if self._executing:
                if pause_signal_sent:
                    pause_signal_sent = False
                    start_time = time.time()

                twist_msg.linear.x = self.vel_x
                twist_msg.linear.y = self.vel_y
                twist_msg.angular.z = self.vel_theta
                self.vel_pub.publish(twist_msg)
                elapsed = elapsed_before_pause + (time.time() - start_time)
            else:
                if not pause_signal_sent:
                    pause_signal_sent = True
                    elapsed_before_pause = elapsed
                    start_time = time.time()

                    twist_msg.linear.x = 0.
                    twist_msg.linear.y = 0.
                    twist_msg.angular.z = 0.
                    self.vel_pub.publish(twist_msg)
            rospy.sleep(0.05)

        print('[{0}] Stopping motion'.format(self.name))
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.vel_pub.publish(twist_msg)

        self._executing = False
        if self._preempted:
            self._preempted = False
