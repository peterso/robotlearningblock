from action_msgs.msg import GoalStatus
from robothon_taskboard_msgs.action import ExecuteTask
from robothon_taskboard_msgs.msg import Task
from robothon_taskboard_msgs.msg import TaskStep
from robothon_taskboard_msgs.msg import SensorMeasurement

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RobothonTaskBoardGoalSender(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, ExecuteTask, 'taskboard_execute_task')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        message = 'Feedback: {0}: '.format(feedback.feedback.elapsed_time)
        self.get_logger().info('Received feedback: {0}'.format(message))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.finish_time))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = ExecuteTask.Goal()
        goal_msg.human_task = False

        task = Task()
        task.name = "Default task"

        goal_msg.task = task

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        self.get_logger().info('Cancelling goal')
        self._send_goal_future.result().cancel_goal_async()

def main(args=None):
    try:
        rclpy.init(args=args)

        goal_sender = RobothonTaskBoardGoalSender()

        goal_sender.send_goal()

        rclpy.spin(goal_sender)
    except (KeyboardInterrupt, ExternalShutdownException):
        goal_sender.cancel_goal()


if __name__ == '__main__':
    main()
