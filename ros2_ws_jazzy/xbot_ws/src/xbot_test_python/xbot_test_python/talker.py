"""

"""
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('my_publisher')

        self.publisher_ = self.create_publisher(
            String, 'chatter', 10
        )
        self.timer = self.create_timer(
            0.5, self.timer_callback
        )

        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2 : {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destory_node()
        rclpy.shutdown()        

if __name__ == '__main__':
    main();        


