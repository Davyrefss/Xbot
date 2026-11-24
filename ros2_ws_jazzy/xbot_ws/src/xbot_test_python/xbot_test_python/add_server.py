"""

"""

import rclpy
from rclpy.node import Node 
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('my_server')

        self.service = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_callback
        )
        self.get_logger().info('Add Server is ready!')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'{request.a} + {request.b} = {response.sum}'
        )     

        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
