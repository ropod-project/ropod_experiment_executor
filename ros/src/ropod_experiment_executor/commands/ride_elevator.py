from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, TaskProgressELEVATOR, Status
from ropod_ros_msgs.msg import ExecuteExperimentFeedback
from ropod_experiment_executor.commands.command_base import CommandBase

class RideElevator(CommandBase):
    '''A state taking care of riding an elevator.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, experiment_server, **kwargs):
        super(RideElevator, self).__init__(name, experiment_server,
                                           outcomes=['done', 'failed'])

        self.destination_floor = kwargs.get('destination_floor', -1)
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.ride_elevator_action_topic = kwargs.get('ride_elevator_action_topic',
                                                     '/ropod_task_executor/RIDE_ELEVATOR')
        self.elevator_progress_topic = kwargs.get('elevator_progress_topic',
                                                  '/task_progress/elevator')

        self.ride_elevator_pub = rospy.Publisher(self.ride_elevator_action_topic,
                                                 Action, queue_size=5)
        self.elevator_progress_sub = rospy.Subscriber(self.elevator_progress_topic,
                                                      TaskProgressELEVATOR,
                                                      self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publishers time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends a RIDE_ELEVATOR action and waits for it to finish. Returns "done" if
        the action finishes within "self.timeout_s" seconds; returns "failed" otherwise.
        '''
        feedback_msg = ExecuteExperimentFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = ExecuteExperimentFeedback.ONGOING

        action_msg = Action()
        action_msg.type = 'RIDE_ELEVATOR'
        action_msg.goal_floor = self.destination_floor

        print('[{0}] Riding elevator to floor {1}'.format(self.name, self.destination_floor))
        self.ride_elevator_pub.publish(action_msg)
        self.__wait_for_action(feedback_msg)

        # if the RIDE_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if not self.action_completed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator ride did not finish within the alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.elevator_progress_sub.unregister()
        return 'done'

    def action_progress_cb(self, progress_msg):
        '''Processes an elevator action progress message and modifies the value of
        self.action_completed depending on the message status code.
        '''
        if progress_msg.status.status_code == Status.GOAL_REACHED:
            self.action_completed = True

    def __wait_for_action(self, feedback_msg):
        '''Waits for an action to either complete (self.action_completed
        will be set if that happens) or timeout (as specified by self.timeout_s);
        publishes periodic feedback messages about the progress as well.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.ExecuteExperimentFeedback -- a feedback message prefilled
                      with the command name and state

        '''
        self.action_completed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and not self.action_completed:
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)
        return self.action_completed

    def __report_failure(self, feedback_msg, error_str):
        '''Publishes a command feedbak message and sends a state info message.

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.ExecuteExperimentFeedback -- a feedback message prefilled
                      with the command name and state
        error_str: str -- an error string to be printed on screen

        '''
        print(error_str)
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = ExecuteExperimentFeedback.FAILED
        self.send_feedback(feedback_msg)
