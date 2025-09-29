from ex03_01.srv import SummFullName
import rclpy
from rclpy.node import Node

class FullNameService(Node):
    def __init__(self):
        super().__init__('full_name_service')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.full_name_callback)

    def full_name_callback(self, request, response):
        response.full_name = f"{request.surname} {request.name} {request.patronymic}"
        self.get_logger().info(f'Incoming request\nSurname: {request.surname} Name: {request.name} Patronymic: {request.patronymic}')
        return response

def main():
    rclpy.init()
    full_name_service = FullNameService()
    rclpy.spin(full_name_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()