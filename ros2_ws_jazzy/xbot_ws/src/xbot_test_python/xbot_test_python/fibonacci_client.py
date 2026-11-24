"""
action client -> Fibonacci
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci

class FibonacciClient(Node):
    def __init__(self):
        super().__init__('my_action_client')
        self.action_client = ActionClient(
            self, Fibonacci, 'fibonacci'
        )
        self.get_logger().info('Fibonacci Client is ready!')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {list(feedback.partial_sequence)}')

    def send_goal(self, order):
        goal = Fibonacci.Goal()
        goal.order = order
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal, feedback_callback = self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()

        if response is not None:
            self.get_logger().info(f'Result: {response.result.sequence}')
            self.get_logger().info(f'Goal status  code: {response.status}')


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciClient()

    try:
        node.send_goal(5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()                        