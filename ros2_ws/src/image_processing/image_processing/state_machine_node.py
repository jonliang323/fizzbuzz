import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import board
import busio
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import MotorCommand

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_subscriber')
        self.clock = Clock()
        self.prev_time = self.clock.now()

        self.FOV_XY = 480,680
        self.MAX_SPEED = 100
        self.NORM_SPEED = 50
        self.MAX_DELTA = self.MAX_SPEED - self.NORM_SPEED

        self.block_align = False
        self.prev_align_error = 0
        self.error_integralL, self.error_integralR = 0,0
        #gains should result in critical damping, best response
        self.p_gainL, self.i_gainL, self.d_gainL = 1,0,0
        self.p_gainR, self.i_gainR, self.d_gainR = 1,0,0

        # 360 scan variables
        self.scan_360_active = False
        self.detected_objects = []
        self.current_angle = 0.0
        self.imu_angle_start = None
        self.spin_speed = 30 #30 is placeholder 




        self.state_machine_sub = self.create_subscription(CubeTracking, "cube_location_info", self.state_machine_callback, 10)

        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

        #imu initialization
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not spi.try_lock():
            pass

        spi.configure(baudrate=5000000)

        self.imu = ICM42688(spi)
        self.imu.begin()

        self.start_360_scan()


    def state_machine_callback(self, msg: CubeTracking):
        dT = (self.clock.now() - self.prev_time).nanoseconds/1e9
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel
        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        
        if self.scan_360_active:
            # print("scan active")
            #read imu angle
            # imu_angle = self.get_current_imu_angle() 
            # if self.imu_angle_start is not None:
            #     self.current_angle = (imu_angle - self.imu_angle_start) % 360

            # #if object is detected at center
            if msg.x_center == 340:
                self.current_angle += self.get_delta_imu_angle()
                self.prev_time = self.clock.now()
                cube_angle = self.current_angle
                cube_distance = msg.distance
                self.detected_objects.append((cube_angle, cube_distance))
                print(f'object detected at {cube_angle} a distance of {cube_distance} away.')

                
                
            
            # #if scan is complete
            if self.current_angle >= 350:
                self.scan_360_active = False
                closest_object = self.handle_scan_results()
                print(f'scan complete is {self.scan_360_active} and the closest object is {closest_object[1]} away at an angle of {closest_object[0]}')
                angle_diff = (self.current_angle - closest_object[0]) % 360

                while angle_diff > 5: #placeholder
                    dc.left_speed = -self.spin_speed
                    dc.right_speed = self.spin_speed
                    self.current_angle += self.get_delta_imu_angle()
                    angle_diff = (self.current_angle - closest_object[0]) % 360

                self.block_align = True



            #robot spins
            dc.left_speed = -self.spin_speed
            dc.right_speed = self.spin_speed
            # print(msg.x_center)
            # print(f'left: {dc.left_speed}, right:{dc.right_speed}')
            


        #Drive until condition is met
        if self.drive_condition(msg.distance):
            norm_speed = self.NORM_SPEED

        # PID alignment, while still or driving; cube to align to depends on msg.center_x
        if self.block_align:
            #no see block, drive straight
            if msg.x_center is not None:
                cur_align_error = self.FOV_XY[0]//2 - msg.x_center
                #capped integral terms
                self.error_integralL = min(self.error_integralL + cur_align_error*dT, self.MAX_DELTA/self.i_gainL)
                self.error_integralR = min(self.error_integralR + cur_align_error*dT, self.MAX_DELTA/self.i_gainR)
                error_deriv = (cur_align_error - self.prev_align_error)/dT

                if self.error_integralL == self.MAX_DELTA/self.i_gainL:
                    print("Left integral saturated")
                if self.error_integralR == self.MAX_DELTA/self.i_gainR:
                    print("Right integral saturated")
                
                #capped deltas
                deltaL = min(self.p_gainL * cur_align_error + self.i_gainL * self.error_integralL + self.d_gainL * error_deriv, self.MAX_DELTA)
                deltaR = min(self.p_gainR * cur_align_error + self.i_gainR * self.error_integralR + self.d_gainR * error_deriv, self.MAX_DELTA)

                self.prev_align_error = cur_align_error
        
        # dc.left_speed = norm_speed + deltaL
        # dc.right_speed = norm_speed + deltaR
        # dc.left_speed = self.NORM_SPEED + deltaL
        # dc.right_speed = self.NORM_SPEED + deltaR

        self.prev_time = self.clock.now() #in case state_machine takes a bit to run
        self.motor_pub.publish(motor_msg)

    def start_360_scan(self):
        self.scan_360_active = True
        self.detected_objects = []
        self.current_angle = 0.0
    
    def handle_scan_results(self):
        #find closest object
        closest_object = None
        min_distance = float('inf')
        for obj in self.detected_objects: #detected_objects is [angle, distance]
            if obj[1] < min_distance:  
                min_distance = obj[1]
                closest_object = obj
        return closest_object
        

    #def activate_elevator


    def get_delta_imu_angle(self):
        cur_time = self.clock.now()
        dT = (cur_time - self.prev_time).nanoseconds/1e9
        accel, gyro = self.imu.get_data()
        rotation_z = gyro[2]
        imu_angle_read = rotation_z * dT
        return imu_angle_read
    

        
    def drive_condition(self, cube_dist):
        #can be more complex
        return cube_dist > 2
    

def main(args=None):
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

