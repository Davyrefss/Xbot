"""

"""

import sys
import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('my_client')

        self.client = self.create_client(
            AddTwoInts, 'add_two_ints'
        )     

    def send_request(self, a, b):
        # waiting for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        # create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

def main(args=None):
    rclpy.init(args=args)
    node = AddClient()

    try:
        a = int(sys.argv[1]) if len(sys.argv) > 1 else 10
        b = int(sys.argv[2]) if len(sys.argv) > 2 else 20
        response = node.send_request(a, b)
        if response is not None:
            node.get_logger().info(
                f'{a} + {b} = {response.sum}'
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()             
