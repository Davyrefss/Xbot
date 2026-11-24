"""

"""

import time
import rclpy
from rclpy.node import Node 
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('my_action_server')

        self.action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.execute_callback
        )

        self.get_logger().info('Fibonacci Server is ready!')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')   
        order = goal_handle.request.order
        result = Fibonacci.Result()

        if order < 0:
            self.get_logger().warn('Order must be positive')
            goal_handle.abort()
            return Fibonacci.Result()

        sequence = []
        feedback = Fibonacci.Feedback()

        for i in range(order):
            if (i == 0):
                sequence.append(0)
            elif (i == 1):
                sequence.append(1)
            else:
                sequence.append(sequence[-1] + sequence[-2])

            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Feedback: {sequence}')
            time.sleep(1) 

        # 标记任务成功
        goal_handle.succeed()   

        result.sequence = sequence
        return result

    
def main(args=None):
    rclpy.init(args=args)
    node = FibonacciServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


            
