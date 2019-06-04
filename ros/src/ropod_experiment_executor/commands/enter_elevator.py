from __future__ import print_function
import time
import rospy

from ropod_ros_msgs.msg import Action, TaskProgressELEVATOR, Status
from ropod_ros_msgs.msg import CommandFeedback, StateInfo
from ropod_experiment_executor.commands.command_base import CommandBase

class EnterElevator(CommandBase):
    '''A state taking care of waiting for an elevator and entering it.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, name, **kwargs):
        super(EnterElevator, self).__init__(name, outcomes=['done', 'failed'])

        self.areas = kwargs.get('areas', list())
        self.area_floor = kwargs.get('area_floor', 0)
        self.elevator_id = kwargs.get('elevator_id', 1)
        self.elevator_door_id = kwargs.get('elevator_door_id', 1)
        self.timeout_s = kwargs.get('timeout_s', 120.)
        self.wait_for_elevator_action_topic = kwargs.get('wait_for_elevator_action_topic',
                                                         '/ropod_task_executor/WAIT_FOR_ELEVATOR')
        self.enter_elevator_action_topic = kwargs.get('enter_elevator_action_topic',
                                                      '/ropod_task_executor/ENTER_ELEVATOR')
        self.elevator_progress_topic = kwargs.get('elevator_progress_topic',
                                                  '/task_progress/elevator')
        self.wait_for_elevator_pub = rospy.Publisher(self.wait_for_elevator_action_topic,
                                                     Action, queue_size=5)
        self.enter_elevator_pub = rospy.Publisher(self.enter_elevator_action_topic,
                                                  Action, queue_size=5)
        self.elevator_progress_sub = rospy.Subscriber(self.elevator_progress_topic,
                                                      TaskProgressELEVATOR,
                                                      self.action_progress_cb)
        self.action_completed = False

        # wait for a while to give the action publishers time to initialise
        rospy.sleep(1.)

    def execute(self, userdata):
        '''Sends a WAIT_FOR_ELEVATOR action and waits for it to finish;
        if the elevator arrives within "self.timeout_s" seconds,
        sends an ENTER_ELEVATOR action and waits for it to finish.
        Stops the experiment if either of the actions fails.
        '''
        feedback_msg = CommandFeedback()
        feedback_msg.command_name = self.name
        feedback_msg.state = CommandFeedback.ONGOING

        # we first send a WAIT_FOR_ELEVATOR action to the robot
        # so that it can wait for the elevator to arrive
        action_msg = Action()
        action_msg.type = 'WAIT_FOR_ELEVATOR'
        action_msg.elevator.elevator_id = self.elevator_id
        action_msg.elevator.door_id = self.elevator_door_id
        action_msg.start_floor = self.area_floor
        action_msg.goal_floor = self.area_floor

        print('[{0}] Waiting for elevator {1} at door {2}'.format(self.name,
                                                                  self.elevator_id,
                                                                  self.elevator_door_id))
        self.wait_for_elevator_pub.publish(action_msg)
        self.__wait_for_action(feedback_msg)

        # if the WAIT_FOR_ELEVATOR action could not be completed within the alloted
        # time, we send a failure feedback message and stop the experiment
        if not self.action_completed:
            self.__report_failure(feedback_msg,
                                  '[{0}] Elevator did not arrive within the alloted time; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        # if the WAIT_FOR_ELEVATOR action ends in success, we proceed
        # by sending an ENTER_ELEVATOR action to the robot
        action_msg = Action()
        action_msg.type = 'ENTER_ELEVATOR'
        action_msg.start_floor = self.area_floor
        action_msg.goal_floor = self.area_floor
        self.action_completed = False

        print('[{0}] Entering elevator {1} at door {2}'.format(self.name,
                                                               self.elevator_id,
                                                               self.elevator_door_id))
        self.enter_elevator_pub.publish(action_msg)
        self.__wait_for_action(feedback_msg)

        # if the ENTER_ELEVATOR action could not be completed successfully,
        # we send a failure feedback message and stop the experiment
        if not self.action_completed:
            self.__report_failure(feedback_msg,
                              '[{0}] Could not enter elevator; giving up'.format(self.name))
            self.elevator_progress_sub.unregister()
            return 'failed'

        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FINISHED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.SUCCESS)
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
        feedback_msg: ropod_ros_msgs.CommandFeedback -- a feedback message prefilled
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
        feedback_msg: ropod_ros_msgs.CommandFeedback -- a feedback message prefilled
                      with the command name and state
        error_str: str -- an error string to be printed on screen

        '''
        print(error_str)
        feedback_msg.stamp = rospy.Time.now()
        feedback_msg.state = CommandFeedback.FAILED
        self.send_feedback(feedback_msg)
        self.send_state(StateInfo.ERROR)
