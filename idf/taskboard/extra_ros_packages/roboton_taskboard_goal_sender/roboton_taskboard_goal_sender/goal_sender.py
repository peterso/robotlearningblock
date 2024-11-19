from action_msgs.msg import GoalStatus
from roboton_taskboard_msgs.action import ExecuteTask
from roboton_taskboard_msgs.msg import Task
from roboton_taskboard_msgs.msg import TaskStep
from roboton_taskboard_msgs.msg import SensorMeasurement

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RobotonTaskBoardGoalSender(Node):

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
        task.name = "Test task"

        # Press Blue Button
        task_step = TaskStep()
        task_step.sensor_name = "BLUE_BUTTON"
        task_step.type = TaskStep.TASK_STEP_TYPE_EQUAL
        task_step.target.type = SensorMeasurement.SENSOR_MEASUREMENT_TYPE_BOOL
        task_step.target.bool_value.append(True)
        task.steps.append(task_step)

        # Move fader to 0.5
        task_step = TaskStep()
        task_step.sensor_name = "FADER"
        task_step.type = TaskStep.TASK_STEP_TYPE_EQUAL
        task_step.target.type = SensorMeasurement.SENSOR_MEASUREMENT_TYPE_ANALOG
        task_step.target.analog_value.append(0.5)
        task_step.tolerance = 0.1
        task.steps.append(task_step)

        # Press Red Button
        task_step = TaskStep()
        task_step.sensor_name = "RED_BUTTON"
        task_step.type = TaskStep.TASK_STEP_TYPE_EQUAL
        task_step.target.type = SensorMeasurement.SENSOR_MEASUREMENT_TYPE_BOOL
        task_step.target.bool_value.append(True)
        task.steps.append(task_step)

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

        goal_sender = RobotonTaskBoardGoalSender()

        goal_sender.send_goal()

        rclpy.spin(goal_sender)
    except (KeyboardInterrupt, ExternalShutdownException):
        goal_sender.cancel_goal()


if __name__ == '__main__':
    main()
