import rclpy
from rclpy.node import Node
from raven import Raven
from image_processing_interfaces.msg import MotorCommand

class RavenNode(Node):
    def __init__(self):
        super().__init__('raven_subscriber')
        self.raven_board = Raven()
        # Set motors to DIRECT
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH2, Raven.MotorMode.DIRECT)

        self.motor_sub = self.create_subscription(MotorCommand, "motor_command", self.raven_callback, 10)

    def raven_callback(self, msg: MotorCommand):
        # Speed controlled:
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 100) # Let the motor use all the torque to get to speed factor
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, msg.left_speed)
        
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH2, 100)
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH2, msg.right_speed)

        # Torque controlled:
        # self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 100) # Make motor try to run at max speed forward
        # self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 10) # Let it use up to 10% available torque