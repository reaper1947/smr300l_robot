#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from next2_msgs.action import SerialOperation 
from rclpy.task import Future

class SerialOperationClient(Node):
    def __init__(self):
        super().__init__('serial_operation_client')
        self.client = ActionClient(self, SerialOperation, 'serial_operation')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        goal_msg = SerialOperation.Goal()
        goal_msg.cmd = "serial_send"
        goal_msg.arg_sr_send = "on"
        goal_msg.arg_sr_send_repeats = True
        goal_msg.timeout = 3
        goal_msg.arg_string = 'uuuuu'
        goal_msg.arg_int = 3
        goal_msg.arg_strings = ["on", "go", "uuuuu", "d", "e3ee", "RxID"]

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: sequence={feedback.sequence}")

    def result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Result received: result={result.result}, text={result.text}, sequence={result.sequence}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = SerialOperationClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
