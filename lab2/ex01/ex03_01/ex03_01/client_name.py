import sys
from ex03_01.srv import SummFullName
import rclpy
from rclpy.node import Node

class FullNameClient(Node):
    def __init__(self):
        super().__init__('full_name_client')
        self.cli = self.create_client(SummFullName, 'SummFullName')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SummFullName.Request()

    def send_request(self, surname, name, patronymic):
        self.req.surname = surname
        self.req.name = name
        self.req.patronymic = patronymic
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    client = FullNameClient()
    if len(sys.argv) != 4:
        client.get_logger().error('Please provide surname, name, and patronymic as arguments')
        return
    future = client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(f'Result: {response.full_name}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()