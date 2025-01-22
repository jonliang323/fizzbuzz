import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import time
import board
import busio
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import EncoderCounts
from image_processing_interfaces.msg import MotorCommand

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.dT = 0.1 #seconds
        self.clock = Clock()

        self.FOV_XY = 480,680
        self.MAX_SPEED = 100
        self.NORM_SPEED = 50
        self.MAX_DELTA = self.MAX_SPEED - self.NORM_SPEED
        self.WHEEL_RADIUS = 2*0.0254 #meters
        self.DIAMETER = 8*0.0254 #meters
        self.CLASSES = ['green','red','sphere']

        #cv variables
        self.closest_obj = None
        #encoder variables
        self.encoderL = 0
        self.encoderR = 0

        #angle variables
        self.cur_left_speed = 0
        self.cur_right_speed = 0
        self.current_angle = 0.0
        self.turn_angle = 0

        self.block_align = False
        self.prev_align_error = 0
        self.error_integralL, self.error_integralR = 0,0
        #gains should result in critical damping, best response
        self.p_gainL, self.i_gainL, self.d_gainL = 1,0,0
        self.p_gainR, self.i_gainR, self.d_gainR = 1,0,0

        # 360 scan variables
        self.scan_360_active = False
        self.scanned = False
        self.scan_count = 0
        self.detected_objects = []
        self.spin_speed = 30 #30 is placeholder 
        self.scan_timer = self.clock.now()

        #elevator variables
        self.block_intake = False
        self.first_sphere_grabbed = False
        self.red_counter = 0
        #self.elevator_timer = self.create_timer(0.5, self.activate_elevator)
        #self.elevator_timer_counter = 0
        self.elevator_state = 'idle'

        self.cv_sub = self.create_subscription(CubeTracking, "cube_location_info", self.cv_callback, 10)
        self.encoder_sub = self.create_subscription(EncoderCounts, "encoder_info", self.encoder_callback, 10)
        self.state_machine = self.create_timer(self.dT, self.state_machine_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

        #imu initialization
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not spi.try_lock():
            pass

        spi.configure(baudrate=5000000)

        self.imu = ICM42688(spi)
        self.imu.begin()

        self.start_360_scan()
    
    def cv_callback(self, msg: CubeTracking):
        distances = msg.distances
        x_centers = msg.x_centers
        obj_type = msg.obj_types
        closest = self.find_closest_index(distances)
        self.closest_obj = (distances[closest], x_centers[closest], obj_type[closest])


    def encoder_callback(self, msg: EncoderCounts):
        self.encoderL = msg.encoder1
        self.encoderR = msg.encoder2

    def state_machine_callback(self):        
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel
        self.update_angle(self.dT)
        
        if self.scan_360_active:
            self.turn_angle = 90
            # #if object is detected at center
            # if abs(msg.x_center - 320) <= 40:
            #     # self.scan_360_active = False
            #     # dc.left_speed = 0
            #     # dc.right_speed = 0
            #     self.prev_time = self.clock.now()
            #     cube_angle = self.current_angle 
            #     cube_distance = msg.distance
            #     self.detected_objects.append((cube_angle, cube_distance))
            #     print(f'\n\n\n\n object detected at {cube_angle} a distance of {cube_distance} away.')

            if self.scan_count < 4:
                if not self.scanned:
                    #snaphsot the field, each object is a stack until knocking over
                    #distance, angle of detection (ideally 0, 90, 180, 270)
                    cur_angle = self.scan_count*90
                    self.detected_objects.append(self.closest_obj, cur_angle) #maybe integrate with new angle calculation
                    self.scanned = True
                #time for 90 degree turn, can be adjusted for proper angle turn
                #if (self.clock.now() - self.scan_timer).nanoseconds/1e9 < 0.5:
                if self.current_angle < self.turn_angle:
                    deltaL = -self.spin_speed
                    deltaR = self.spin_speed
                else:
                    scanned = False
                    self.turn_angle += 90
                    self.scan_count += 1
            else: #if scan is complete
                self.scan_360_active = False
                deltaL = 0
                deltaR = 0
                closest = self.find_closest_index([obj[0] for obj in self.detected_objects])
                self.target = self.detected_objects[closest]
                print(f'\n\n scan complete, closest object is {self.CLASSES[self.target[2]]},\n{self.target[0]} away at an angle of...')
                #angle_diff = (self.current_angle - closest_object[0]) % 360
                print(f'\n\n all detected_objects: {self.detected_objects} \n\n\n\n') 
                self.block_align = True

        #PID align to angle, then use x_center? or just angle?
        # PID alignment, while still or driving; cube to align to should depend on recorded angle
        #if self.block_align:
            # if msg.x_center is not None:
            #     cur_align_error = self.FOV_XY[0]//2 - msg.x_center
            #     #capped integral terms
            #     self.error_integralL = min(self.error_integralL + cur_align_error*dT, self.MAX_DELTA/self.i_gainL)
            #     self.error_integralR = min(self.error_integralR + cur_align_error*dT, self.MAX_DELTA/self.i_gainR)
            #     error_deriv = (cur_align_error - self.prev_align_error)/dT

            #     if self.error_integralL == self.MAX_DELTA/self.i_gainL:
            #         print("Left integral saturated")
            #     if self.error_integralR == self.MAX_DELTA/self.i_gainR:
            #         print("Right integral saturated")
                
            #     #capped deltas
            #     deltaL = min(self.p_gainL * cur_align_error + self.i_gainL * self.error_integralL + self.d_gainL * error_deriv, self.MAX_DELTA)
            #     deltaR = min(self.p_gainR * cur_align_error + self.i_gainR * self.error_integralR + self.d_gainR * error_deriv, self.MAX_DELTA)

            #     self.prev_align_error = cur_align_error

        #Drive until condition is met
        # if self.drive_condition(msg.distance):
        #     norm_speed = self.NORM_SPEED
        # else: #stop
        #     norm_speed = 0
        #     self.block_intake = True

        # if self.block_intake:
        #     if block == green or self.first_sphere_grabbed == False:
        #         self.activate_elevator()
        #         self.first_sphere_grabbed = True
        #     elif block == red:
        #         self.activate_bird()
        #         self.red_counter += 1
        #     else:
        #         self.activate_bird()

        # if self.red_counter == 5:
        #     self.activate_dumptruck()

        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        servo = motor_msg.actuate_motors
        #change servo speeds
        dc.left_speed = norm_speed + deltaL
        dc.right_speed = norm_speed + deltaR
        self.cur_left_speed = dc.left_speed
        self.cur_right_speed = dc.right_speed
        self.motor_pub.publish(motor_msg)

        self.prev_time = self.clock.now() #in case state_machine takes a bit to run

    def start_360_scan(self):
        self.scan_360_active = True
        self.detected_objects = []
    
    def find_closest_index(self, distances):
        #find closest distance index
        min = 0
        for i in range(1, len(distances)): #detected_objects is [type, distance, angle of view]
            if distances[i] < distances[min]:  
                min = i
        return min
    
    def update_angle(self, dT):
        alpha_wL, alpha_wR = 1, 1
        beta_factor = 0.4
        _, gyro = self.imu.get_data()
        wZ = gyro[2]
        diff_wheel_x1 = (alpha_wR*self.cur_wR - alpha_wL*self.cur_wL)*self.WHEEL_RAD*dT
        diff_wheel_x2 = (self.encoderR - self.encoderL)/440*2*math.PI*self.WHEEL_RAD
        delta_theta = beta_factor*wZ*dT + (1 - beta_factor) * math.arcsin((diff_wheel_x1+diff_wheel_x2)/2/self.DIAMETER)
        self.current_angle += delta_theta*180/math.PI #degrees

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

