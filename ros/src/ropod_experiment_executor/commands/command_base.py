from __future__ import print_function
import time
import rospy
import smach

class CommandBase(smach.State):
    def __init__(self, name, experiment_server, outcomes,
                 input_keys=list(), output_keys=list()):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.sm_id = ''
        self.name = name
        self.experiment_server = experiment_server
        self.retry_count = 0
        self.executing = False
        self.succeeded = False
        self.robot_name = ''

        self.timeout_s = 0.
        self.action_server = None
        self.action_completed = False
        self.action_failed = False

    def execute(self, userdata):
        raise NotImplementedError('"execute" has to be implemented by child classes')

    def wait_for_action_result(self, feedback_msg):
        '''Waits for an action to either complete (self.action_completed
        will be set if that happens) or timeout (as specified by self.timeout_s);
        publishes periodic feedback messages about the progress as well.
        Returns a Boolean indicating the result of the action (success or failure).

        Keyword arguments:
        feedback_msg: ropod_ros_msgs.ExecuteExperimentFeedback -- a feedback message prefilled
                      with the command name and state

        '''
        self.action_completed = False
        self.action_failed = False
        elapsed = 0.
        start_time = time.time()
        while elapsed < self.timeout_s and \
              not self.action_completed and \
              not self.experiment_server.is_preempt_requested():
            feedback_msg.stamp = rospy.Time.now()
            self.send_feedback(feedback_msg)
            elapsed = time.time() - start_time
            rospy.sleep(0.05)

        # even though we've received feedback that the action
        # has completed, we wait for the action server to complete
        # (in order to prevent other calls to it before it stops)
        # and then take its result as the ultimate action result
        if self.action_completed or self.action_failed:
            self.action_server.wait_for_result()
            result = self.action_server.get_result()
            if not result.success:
                self.action_completed = False
        return self.action_completed

    def send_feedback(self, feedback_msg):
        self.experiment_server.publish_feedback(feedback_msg)
