import rclpy
from rclpy.node import Node
from image_processing_interfaces.msg import CubeTracking 

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_subscriber')
        self.state_machine_sub = self.create_subscription(CubeTracking, "cube_location_info", self.state_machine_callback, 10)

    def state_machine_callback(self, msg: CubeTracking):


def main(args=None):
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

