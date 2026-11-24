"""

"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class SpeedConfig(Node):
    def __init__(self):
        super().__init__('speed_config')

        self.declare_parameter('max_speed', 1.0)
        self.timer = self.create_timer(
            2.0, self.timer_callback
        )

    def timer_callback(self):
        speed = self.get_parameter('max_speed').value
        self.get_logger().info(f'Max speed is: {speed}m/s')


def main(args=None):
    rclpy.init(args=args)
    node = SpeedConfig()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()                  

