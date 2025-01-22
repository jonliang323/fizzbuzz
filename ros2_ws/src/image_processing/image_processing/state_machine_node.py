import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import time
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
        self.scan_timer = self.clock.now()

        #elevator variables
        self.block_intake = False
        self.first_sphere_grabbed = False
        self.red_counter = 0
        self.elevator_timer = self.create_timer(0.5, self.activate_elevator)
        self.elevator_timer_counter = 0
        self.elevator_state = 'idle'




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
            self.current_angle += self.get_delta_imu_angle(dT)

            # #if object is detected at center
            if abs(msg.x_center - 320) <= 40:
                # self.scan_360_active = False
                # dc.left_speed = 0
                # dc.right_speed = 0
                self.prev_time = self.clock.now()
                cube_angle = self.current_angle 
                cube_distance = msg.distance
                self.detected_objects.append((cube_angle, cube_distance))
                print(f'\n\n\n\n object detected at {cube_angle} a distance of {cube_distance} away.')

            #find the closest object in the current FOV
            closest_object = self.handle_scan_results(msg.list_of_x_centers)

            #turn for x amount of time
            turn_duration = 0.5
            while (self.clock.now() - self.scan_timer).nanoseconds/1e9 < turn_duration:
                dc.left_speed = -self.spin_speed
                dc.right_speed = self.spin_speed
                self.motor_pub.publish(motor_msg)
                rclpy.spin_once(self, timeout_sec=0.1)

            #scan for the closest object in the new field of vision
            self.current_angle += self.get_delta_imu_angle(turn_duration)
            self.detected_objects.append((self.current_angle, closest_object.distance))

            # print(self.current_angle)
        
            
            # #if scan is complete
            if (self.clock.now() - self.scan_timer).nanoseconds/1e9 > 2.6:
                self.scan_360_active = False
                dc.left_speed = 0
                dc.right_speed = 0
                closest_object = self.handle_scan_results()
                print(f'\n\n scan complete is {self.scan_360_active} and the closest object is {closest_object[1]} away at an angle of {closest_object[0]}')
                angle_diff = (self.current_angle - closest_object[0]) % 360
                print(f'\n\n detected_objects: {self.detected_objects} \n\n\n\n') 


                # self.block_align = True


            #robot spins
            dc.left_speed = -self.spin_speed
            dc.right_speed = self.spin_speed
                
            # print(f'msg.x_center:{msg.x_center}')
            # print(f'left: {dc.left_speed}, right:{dc.right_speed}')
            


        #Drive until condition is met
        if self.drive_condition(msg.distance):
            norm_speed = self.NORM_SPEED
        else: #stop
            norm_speed = 0
            self.block_intake = True

        # PID alignment, while still or driving; cube to align to should depend on recorded angle
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
        
        dc.left_speed = norm_speed + deltaL
        dc.right_speed = norm_speed + deltaR


        if self.block_intake:
            if block == green or self.first_sphere_grabbed == False:
                self.activate_elevator()
                self.first_sphere_grabbed = True
            elif block == red:
                self.activate_bird()
                self.red_counter += 1
            else:
                self.activate_bird()

        if self.red_counter == 5:
            self.activate_dumptruck()

        self.prev_time = self.clock.now() #in case state_machine takes a bit to run
        self.motor_pub.publish(motor_msg)
    

    def start_360_scan(self):
        self.scan_360_active = True
        self.detected_objects = []
        self.current_angle = 0.0
    
    def handle_scan_results(self, detected_objects):
        #find closest object
        closest_object = None
        min_distance = float('inf')
        for obj in detected_objects: #detected_objects is [angle, distance]
            if obj[1] < min_distance:  
                min_distance = obj[1]
                closest_object = obj
        return closest_object

    def activate_elevator(self):
        motor_msg = MotorCommand()

        #elevator starts: angle1 = 90 (elevator up) angle2 = -25 (claws closed) angle3 = 0 (flap open)
        self.elevator_state == 'open claws'
        self.elevator_timer_count = 0
        

        if self.elevator_state == 'open claws':
            motor_msg.actuate_motors.angle2 = -40  #claws open
            self.elevator_timer_count += 1

            if self.elevator_timer_count >= 4:  #wait 2 seconds
                self.elevator_state = 'elev move down'
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move down':
            motor_msg.actuate_motors.angle1 = -90  #elevator moves down
            motor_msg.actuate_motors.angle3 = 90   #flap closes

            self.elevator_timer_count += 1
            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'close claws'
                self.elevator_timer_count = 0

        #return elevator to starting position
        elif self.elevator_state == 'close claws':
            motor_msg.actuate_motors.angle2 = -25  #claws close
            self.elevator_timer_count += 1
            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'elev move up'
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move up':
            motor_msg.actuate_motors.angle1 = -90  #elevator moves up while holding block
            motor_msg.actuate_motors.angle3 = 90   #flap opens, ready for next block
            self.elevator_timer_count += 1

            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'idle'
                self.elevator_timer_count = 0


        # Publish motor command
        self.elevator_pub.publish(motor_msg)
        

    def activate_bird(self):
        #activate bird when red block comes
        motor_msg = MotorCommand()

        motor_msg.actuate_motors.angle4 = -90 #idk what the bird angles are

    def activate_dumptruck(self):
        motor_msg = MotorCommand()
        #some way to drive to purple wall
        motor_msg.drive_motors.left_speed = 30 #change this, maybe add something in DCCommand.msg like dt_speed and set to 0 for everything except when dt is activated


    def get_delta_imu_angle(self, time):
        accel, gyro = self.imu.get_data()
        rotation_z = gyro[2]
        imu_angle_read = rotation_z * time
        return imu_angle_read*(180/math.pi)
    

        
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

