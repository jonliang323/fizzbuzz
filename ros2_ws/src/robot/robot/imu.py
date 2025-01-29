import time

import board
import busio
import rclpy
from icm42688 import ICM42688
from rclpy.node import Node
from std_msgs.msg import Float32


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not spi.try_lock():
            pass

        spi.configure(baudrate=5000000)
        self.angle = 0
        self.imu = ICM42688(spi)
        self.imu.begin()

        self.imu.set_gyro_fullscale_odr(ICM42688.GYRO_FS.FS2000dps, ICM42688.ODR.ODR1kHz)
        self.imu.set_accel_fullscale_odr(ICM42688.ACCEL_FS.FS2g, ICM42688.ODR.ODR1kHz)

        self.get_logger().info("imu calibration has started")
        start_time = time.time()
        
        for i in range(1000):
            while time.time() - start_time < i/1000:
                pass
            accel, gyro = self.imu.get_data() 
            self.angle += gyro[2]

        self.offset = self.angle / 1000
        self.angle = 0 
              
        self.imu_pub = self.create_publisher(Float32, 'imu', 10)
        
        self.read_timer = self.create_timer((1/1000), self.imu_read)
        self.pub_timer = self.create_timer(.01, self.imu_publisher)

        self.get_logger().info("imu node has started")

    def imu_read(self):
        accel, gyro = self.imu.get_data() 
        self.angle += (gyro[2]- self.offset) * (1/1000) 

    
    
    def imu_publisher(self):
        self.imu_pub.publish(Float32(data=self.angle))


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down imu node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()