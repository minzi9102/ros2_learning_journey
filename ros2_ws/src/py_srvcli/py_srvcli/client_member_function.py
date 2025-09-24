import sys

# from example_interfaces.srv import AddTwoInts
from tutorial_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()

    def send_request(self, a, b, c):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        self.future = self.cli.call_async(self.req)

#阻塞式处理服务响应
# def main():  
#     rclpy.init()

#     minimal_client = MinimalClientAsync()
#     future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
#     rclpy.spin_until_future_complete(minimal_client, future)
#     response = future.result()
#     minimal_client.get_logger().info(
#         'Result of add_two_ints: for %d + %d + %d = %d' %
#         (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))

#     minimal_client.destroy_node()
#     rclpy.shutdown()

#非阻塞式处理服务响应
def main():

  rclpy.init()

  minimal_client = MinimalClientAsync()
  minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))

  while rclpy.ok():
      rclpy.spin_once(minimal_client)
      if minimal_client.future.done():
          try:
              response = minimal_client.future.result()
          except Exception as e:
              minimal_client.get_logger().info(
                  'Service call failed %r' % (e,))
          else:
              minimal_client.get_logger().info(
                  'Result of add_three_ints: woshinibaba %d + %d + %d = %d' %
                  (int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), response.sum))
          break

  minimal_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()