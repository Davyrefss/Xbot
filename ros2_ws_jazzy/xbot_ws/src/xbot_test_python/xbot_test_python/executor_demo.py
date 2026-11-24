"""

"""

import time 
import threading
import rclpy
from rclpy.node import Node 
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import (
    ReentrantCallbackGroup,
    MutuallyExclusiveCallbackGroup
)

class ExecutorDemo(Node):
    def __init__(self):
        super().__init__('executor_demo')
        # 互斥回调组(一次只允许一个回调执行)
        # self.group = MutuallyExclusiveCallbackGroup()
        # 可重入回调组(可以同时执行多个回调)
        self.group = ReentrantCallbackGroup()
        self.slow_timer = self.create_timer(
            4.0,
            self.slow_callback,
            callback_group = self.group
        )
        self.fast_timer = self.create_timer(
            0.5,
            self.fast_callback,
            callback_group = self.group
        )
        self.counter = 0
        self.lock = threading.Lock()
        self.get_logger().info('Executor Demo is ready!')

    def slow_callback(self):
        time.sleep(2.5)     # 耗时操作
        # with self.lock:
        #     self.counter += 1
        self.counter += 1
        self.get_logger().info(f'slow_callback->counter: {self.counter}')

    def fast_callback(self):
        # with self.lock:
        #     self.counter += 1
        self.counter += 1
        self.get_logger().info(f'fast_callback->counter: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = ExecutorDemo()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)  

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
