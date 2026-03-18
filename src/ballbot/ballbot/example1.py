import rclpy # pyright: ignore[reportMissingImports]
from rclpy.node import Node # type: ignore

class Controller(Node):
    def __init__(self):
        super().__init__('ballbot_controller')
        self.get_logger().info("Ballbot controller started")

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()