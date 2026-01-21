import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node

class ServiceTest(Node):

    def __init__(self):
        super().__init__('server_test')
        self.srv = self.create_service(Trigger, 'check_robot_status', self.service)

    def service(self, request, response):
        self.get_logger().info("receive the request")
        response.success = True
        response.message = "ready"
        return response

def main():
    rclpy.init()
    node = ServiceTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
