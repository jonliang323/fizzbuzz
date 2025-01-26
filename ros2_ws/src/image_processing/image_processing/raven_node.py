import rclpy
from rclpy.node import Node
from raven import Raven
from image_processing_interfaces.msg import MotorCommand
from image_processing_interfaces.msg import EncoderCounts

class RavenNode(Node):
    def __init__(self):
        super().__init__('raven_subscriber')
        self.raven_board = Raven()
        self.dT = 0.1

        # Set motors to DIRECT
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH2, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 15) # Let the motor use 50% max torque to get to speed factor
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH2, 15)

        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0) # Set encoder count for motor 1 to zero
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH2, 0) # Set encoder count for motor 1 to zero

        self.motor_sub = self.create_subscription(MotorCommand, "motor_command", self.raven_callback, 10)
        self.encoder_timer = self.create_timer(self.dT, self.delta_encoder_callback)
        self.delta_encoder_pub = self.create_publisher(EncoderCounts, "delta_encoder_info", 10)

    def raven_callback(self, msg: MotorCommand):
        dc = msg.drive_motors
        servo = msg.actuate_motors
        rev = False
        if dc.left_speed < 0:
            left_speed = abs(dc.left_speed)
            left_rev = True
        else:
            left_speed = dc.left_speed
            left_rev = False
        
        if dc.right_speed < 0:
            right_speed = abs(dc.right_speed)
            right_rev=True
        else:
            right_speed = dc.right_speed
            right_rev=False

        if dc.dt_speed < 0:
            dt_speed = abs(dc.dt_speed)
            dt_rev=True
        else:
            dt_speed = dc.dt_speed
            dt_rev=False

        # print(f"Left Motor - Speed: {left_speed}, Reverse: {left_rev}")
        # print(f"Right Motor - Speed: {right_speed}, Reverse: {right_rev}")

        # Speed controlled:
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, right_speed, reverse=right_rev)
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH2, left_speed, reverse=left_rev)
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH3, dt_speed, reverse=dt_rev)

        # Torque controlled:
        # self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 100) # Make motor try to run at max speed forward
        # self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 10) # Let it use up to 10% available torque

        # Set the servo 1 to x degrees with custom pulse microseconds
        # self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle, servo.min_us, servo.max_us)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle1_elev, 500, 2500)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH2, servo.angle2_claw, 500, 2500)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH3, servo.angle3_flap, 500, 2500)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH4, servo.angle4_duck, 500, 2500)

    def delta_encoder_callback(self):
        delta_encoder_msg = EncoderCounts()
        delta_encoder_msg.encoder1 = -self.raven_board.get_motor_encoder(Raven.MotorChannel.CH1)
        delta_encoder_msg.encoder2 = self.raven_board.get_motor_encoder(Raven.MotorChannel.CH2)
        # self.get_logger().info("entered encoder_callback")
        
        #Reset encoder counts
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH1, 0)
        self.raven_board.set_motor_encoder(Raven.MotorChannel.CH2, 0)

        self.delta_encoder_pub.publish(delta_encoder_msg)


def main(args=None):
    rclpy.init()
    node = RavenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def destroy_node(self):
    # Stop the motors before shutting down
    self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 0, reverse=False)
    self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH2, 0, reverse=False)
    super().destroy_node()  # Call the parent class destroy_node


if __name__ == "__main__":
    main()
