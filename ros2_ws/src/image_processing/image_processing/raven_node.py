import rclpy
from rclpy.node import Node
from raven import Raven
#from icm42688 import ICM42688
#import board
from image_processing_interfaces.msg import MotorCommand
#from image_processing_interfaces.msg import IMUInfo

class RavenNode(Node):
    def __init__(self):
        super().__init__('raven_subscriber')
        self.raven_board = Raven()
        #imu initialization
        #spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        #while not spi.try_lock():
            #pass

        #spi.configure(baudrate=5000000)

        #self.imu = ICM42688(spi)
        #self.imu.begin()

        # Set motors to DIRECT
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH1, Raven.MotorMode.DIRECT)
        self.raven_board.set_motor_mode(Raven.MotorChannel.CH3, Raven.MotorMode.DIRECT)

        self.motor_sub = self.create_subscription(MotorCommand, "motor_command", self.raven_callback, 10)
        #self.imu_pub = self.create_publisher(IMUInfo, "imu_info", 10)

    def raven_callback(self, msg: MotorCommand):
        dc = msg.drive_motors
        servo = msg.actuate_motors

        #print('in the callback')
        # Speed controlled:
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 100) # Let the motor use all the torque to get to speed factor
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, dc.left_speed)
        
        self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH3, 100)
        self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH3, dc.right_speed)

        # Torque controlled:
        # self.raven_board.set_motor_speed_factor(Raven.MotorChannel.CH1, 100) # Make motor try to run at max speed forward
        # self.raven_board.set_motor_torque_factor(Raven.MotorChannel.CH1, 10) # Let it use up to 10% available torque

        # Set the servo 1 to x degrees with custom pulse microseconds
        # self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle, servo.min_us, servo.max_us)
        self.raven_board.set_servo_position(Raven.ServoChannel.CH1, servo.angle, 500, 2500)

        #send imu info on refresh
        #imu_msg = IMUInfo()
        #accel, gyro = self.imu.get_data()
        # Returned linear acceleration is a tuple of (X, Y, Z) m/s^2
        # Returned gyroscope reading is a tuple of (X, Y, Z) radian/s
        #imu_msg.accel = [accel[0],accel[1],accel[2]]
        #imu_msg.gyro = [gyro[0],gyro[1],gyro[2]]
        #self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init()
    node = RavenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
