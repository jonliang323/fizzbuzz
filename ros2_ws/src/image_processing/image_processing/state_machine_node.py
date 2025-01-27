import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import board
import busio
import time
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import EncoderCounts
from image_processing_interfaces.msg import MotorCommand
from std_msgs.msg import Bool

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.dT = 0.01 #seconds
        self.clock = Clock()

        self.FOV_XY = 640,480
        self.FOCAL_X = 888.54513
        self.MAX_SPEED = 100
        self.NORM_SPEED = 50
        self.MAX_DELTA = 100
        self.WHEEL_RADIUS = (3.1875/2)*2.54 #cm
        self.BASE_RADIUS = (10.125/2)*2.54 #cm
        self.CLASSES = ['green','red','sphere']

        #cv variables
        #encoder variables
        self.delta_encoderL = 0
        self.delta_encoderR = 0
        self.ENCODER_RES = 640

        #heading, positioning variables
        self.current_angle = 0.0
        self.turn_angle = 90
        self.current_pos = (0, 0)

        # 360 scan variables
        self.scan_270_active = False
        self.scan = True
        self.detected_objects = []
        self.spin_speed = 50 #30 is placeholder
        self.stack_counter = 0

        #turning state, PID
        self.target_align = False
        self.prev_error_turn = 0
        self.error_integralL_turn, self.error_integralR_turn= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn = 0.1,0.01,0.01
        self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn = 0.1,0.01,0.01

        #turn drive state, PID
        self.target_drive = False
        self.prev_error_drive = 0
        self.error_integralL_drive, self.error_integralR_drive= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive = 0.1,0.01,0.01
        self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive = 0.1,0.01,0.01

        self.block_screen_ratio = 0

        #elevator variables
        self.block_intake = False
        self.first_sphere_grabbed = False
        self.red_counter = 0
        self.elevator_timer_count = 0
        self.elevator = False
        self.dumptruck=False
        self.elevator_state = 'idle'
        self.angle1_elev = 0
        self.angle2_claw = 0
        self.angle3_flap = 0
        self.angle4_duck = 0


        self.cv_sub = self.create_subscription(CubeTracking, "cube_location_info", self.cv_callback, 10)
        self.delta_encoder_sub = self.create_subscription(EncoderCounts, "delta_encoder_info", self.delta_encoder_callback, 10)
        self.state_machine = self.create_timer(self.dT, self.state_machine_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)
        self.scan_pub = self.create_publisher(Bool, "scan_activate", 10)

        #imu initialization
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not spi.try_lock():
            pass

        spi.configure(baudrate=5000000)

        self.imu = ICM42688(spi)
        self.imu.begin()
    
    def cv_callback(self, msg: CubeTracking):
        self.get_logger().info(f'Requested scan')
        #enter here when scan is true, from callback recall initiate
        obj_types = msg.obj_types
        sizes = msg.sizes
        x_centers = msg.x_centers
        block_pixels = msg.block_pixels

        if self.scan_270_active:
            #obj has descriptors: distance, angle, x_center, type
            if obj_types != []:
                closest = self.find_closest_index(sizes)
                rel_angle = int(math.atan((self.FOV_XY[0]/2 - x_centers[closest])/self.FOCAL_X)*180/math.pi)
                closest_obj = {"size":sizes[closest], "angle":self.current_angle + rel_angle, "type":self.CLASSES[obj_types[closest]]}
                self.detected_objects.append(closest_obj)
            #otherwise, detected objects is not changed
        elif self.target_drive:
            #find block screen ratio
            if obj_types != []:
                self.block_screen_ratio = block_pixels/(self.FOV_XY[0]*self.FOV_XY[1])
            else:
                self.block_screen_ratio = 0
        self.scan = False


    def delta_encoder_callback(self, msg: EncoderCounts):
        self.delta_encoderL = msg.encoder1
        self.delta_encoderR = msg.encoder2
        #only update angle here, after encoder deltas have been sent, to be read once per cycle
        self.update_angle_and_pos()
        # self.get_logger().info(f'current_angle: {self.current_angle}')
        # self.get_logger().info(f'current_position: {self.current_pos}')

    def state_machine_callback(self): #called every 0.5 seconds
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel
        
        if self.scan_270_active:
            #spins full 270
            if self.turn_angle <= 270:
                #turns 270 degrees
                #scan is true
                if not self.scan and abs(self.current_angle - self.turn_angle) > 5: #ccw turn, + angle
                    gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
                    gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)

                    angle_err = self.turn_angle - self.current_angle
                    if angle_err > 180:
                        angle_err -= 360
                    elif angle_err < -180:
                        angle_err += 360
                    deltaL, deltaR = self.PID(angle_err, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
                    self.prev_err_turn = angle_err
                elif not self.scan: #self.current_angle matched self.turn_angle, still no scan
                    self.scan = True
                    self.turn_angle += 90
            else: #scan is complete
                deltaL = 0
                deltaR = 0
                #process list
                if len(self.detected_objects) > 0:
                    closest = self.find_closest_index([obj["size"] for obj in self.detected_objects])
                    self.target = self.detected_objects[closest]
                    self.get_logger().info(f'\n\n scan complete, closest object is {self.target["type"]},\nat an angle of {self.target["angle"]}')
                    self.get_logger().info(f'\n\n{len(self.detected_objects)} detected_objects\n\n')
                    self.detected_objects = []
                    self.scan_270_active = False
                    self.target_align = True
                    self.prev_error_turn = 0
                elif len(self.detected_objects) == 0 and self.stack_counter < 5:
                    #TODO: if no cubes found while there are still cubes to be found --> scan again
                    self.turn_angle = 0
                    self.scan = False
                else:
                    self.dumptruck=True

                

        if self.target_align:
            if abs(self.target['angle'] - self.current_angle) > 5:
                #*** turn to face self.target["angle"] here: 
                gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
                gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)

                angle_err = self.target['angle'] - self.current_angle
                if angle_err > 180:
                    angle_err -= 360
                elif angle_err < -180:
                    angle_err += 360
                deltaL, deltaR = self.PID(angle_err, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
                self.prev_err_turn = angle_err
            else:
                #constant scan for cube -> find % cube of screen -> have time delay? sparse scanning
                self.scan = True
                self.target_align = False
                self.target_drive = True
                self.prev_error_turn = 0

        # PID alignment, while still or driving; cube to align to should depend on recorded angle
        if self.target_drive:
            self.get_logger().info(f'block_screen_ratio: {self.block_screen_ratio}')
            if (self.block_screen_ratio < 0.8):
                gainsL = (self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive)
                gainsR = (self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive)

                angle_err = self.target['angle'] - self.current_angle
                if angle_err > 180:
                    angle_err -= 360
                elif angle_err < -180:
                    angle_err += 360
                deltaL, deltaR = self.PID(angle_err, self.prev_error_drive, self.error_integralL_drive, self.error_integralR_drive, gainsL, gainsR)
                norm_speed = self.NORM_SPEED
                self.prev_err_drive = angle_err
            else:
                norm_speed = 0
                self.target_drive = False
                self.block_intake = True
                self.prev_error_drive = 0

        #TODO: block intake, elevator, bird, dumptruck
        #0) grab sphere if not already
        #main sequence:
        #1) add colors to queue, bottom up
        #2) knock over blocks
        #3) intake blocks
        #-> if red, flip flap, activate bird
        #-> if green, flip flap, activate elevator
        #4) scan for objects, find closest, turn to face, drive to object
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

        if self.dumptruck:
            self.activate_dumptruck()
            self.dumptruck = False
        if self.elevator:
            self.activate_elevator()


        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        servo = motor_msg.actuate_motors

        #set motor speeds
        dc.left_speed = int(norm_speed + deltaL)
        dc.right_speed = int(norm_speed + deltaR)
        servo.angle1_elev = self.angle1_elev
        servo.angle2_claw = self.angle2_claw
        servo.angle3_flap = self.angle3_flap
        servo.angle4_duck = self.angle4_duck

        self.motor_pub.publish(motor_msg)

        scan_msg = Bool()
        scan_msg.data = self.scan
        self.scan_pub.publish(scan_msg)
    
    def find_closest_index(self, sizes):
        #find closest distance index (largest size)
        max_s_i = 0
        for i in range(1, len(sizes)):
            if sizes[i] > sizes[min]:  
                max_s_i = i
        return max_s_i
    
    def update_angle_and_pos(self):
        #should be positive or negative
        xL = self.delta_encoderL/self.ENCODER_RES*2*math.pi*self.WHEEL_RADIUS
        xR = self.delta_encoderR/self.ENCODER_RES*2*math.pi*self.WHEEL_RADIUS
        dtheta = (xR-xL)/(2*self.BASE_RADIUS) #rad

        # pos diff calculated before angle update
        #_primes are in rotated reference frame
        if dtheta != 0:  
            dy_prime = (xL/dtheta + self.BASE_RADIUS)*math.sin(dtheta)
            dx_prime = -dy_prime*math.tan(dtheta)
            #rotating to inertial frame given current angle (before update)
            theta = self.current_angle*math.pi/180
            #rotation matrix abt z, applied: [cos -sin; sin cos] * [dx_prime; dy_prime] = [dx; dy]
            #-theta is used to rotate back to inertial frame, unsimplified for readability
            dx = math.cos(-theta)*dx_prime - math.sin(-theta)*dy_prime
            dy = math.sin(-theta)*dx_prime + math.cos(-theta)*dy_prime

            self.current_pos = (self.current_pos[0] - dx*(29/25), self.current_pos[1] + dy*(29/25))
            self.current_angle = self.modulate_angle(self.current_angle + int(dtheta*180/math.pi))

    def modulate_angle(self, angle):
        if angle > 0:
            return angle%360 if angle > 360 else angle
        else:
            return -(abs(angle)%360) if abs(angle) > 360 else angle

    def PID(self, cur_align_error, prev_error, cur_error_integralL, cur_error_integralR, gainsL, gainsR):
        #positive error -> turn left
        #negative error -> turn right
        #capped integral terms
        (p_gainL, i_gainL, d_gainL) = gainsL
        (p_gainR, i_gainR, d_gainR) = gainsR
        error_integralL = max(min(cur_error_integralL + cur_align_error*self.dT, self.MAX_DELTA/i_gainL), -self.MAX_DELTA/i_gainL)
        error_integralR = max(min(cur_error_integralR + cur_align_error*self.dT, self.MAX_DELTA/i_gainR), -self.MAX_DELTA/i_gainR)
        error_deriv = (cur_align_error - prev_error)/self.dT

        if abs(error_integralL) == self.MAX_DELTA/i_gainL:
            self.get_logger().info("Left integral saturated")
        if abs(error_integralR) == self.MAX_DELTA/i_gainR:
            self.get_logger().info("Right integral saturated")
        
        #capped deltas
        deltaL = -max(min(p_gainL * cur_align_error + i_gainL  + d_gainL * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)
        deltaR = max(min(p_gainR * cur_align_error + i_gainR + d_gainR * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)

        return deltaL, deltaR


    def activate_elevator(self):
        if self.elevator == True:      

            if self.elevator_state == 'open claws':
            # if self.i == 0:
                self.angle1_elev = 90
                self.angle3_flap = -90 #flap open...if we want default flap to be closed, we can open it above when we set block_intake to be true
                self.angle2_claw = -45  #claws open
                #robot should move forward at this point. 

                self.elevator_timer_count += 1
                # print(f'state: open claws ------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 200:  #wait 2 seconds
                    self.elevator_state = 'elev move down'
                    # print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move down':
            # elif self.i == 1:
                self.angle1_elev = -90  #elevator moves down
                self.angle3_flap = 0    #flap closes 
                self.angle2_claw = -45
                # print(f'state: elev moving down------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 200:  
                    self.elevator_state = 'close claws'
                    # print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            #return elevator to starting position
            elif self.elevator_state == 'close claws':
            # elif self.i == 2:
                self.angle1_elev = -90
                self.angle3_flap = 0 
                self.angle2_claw = -20  #claws close
                self.elevator_timer_count += 1
                # print(f'state: claws close------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 200:  
                    self.elevator_state = 'elev move up'
                    # print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move up':
            # elif self.i == 3:
                self.angle1_elev = 90  #elevator moves up while holding block
                self.angle3_flap = -90   #flap opens
                self.angle2_claw = -20
                self.elevator_timer_count += 1
                # print(f'state: elev moving up------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 200:  
                    self.elevator_state = 'idle'
                    # print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
            
            if self.elevator_state == 'idle':
            # elif self.i == 4:
                self.angle1_elev = 90  
                self.angle3_flap = 0   
                self.angle2_claw = -20
                # print(f'state: idle ------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')
                # print(f'start angles: elev: {self.angle1_elev}, claw: {self.angle2_claw}, flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'open claws'
                    # print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
                self.elevator = False

        
    #TODO: implement bird
    def activate_bird(self):
        #activate bird when red block comes
        motor_msg = MotorCommand()

        motor_msg.actuate_motors.angle4 = -90 #idk what the bird angles are
        time.sleep(1)
        motor_msg.actuate_motors.angle4 = 90
    
    #
    # TODO: implement dumptruck
    def activate_dumptruck(self):
        motor_msg = MotorCommand()
        #some way to drive to purple wall
        motor_msg.drive_motors.dt_speed = 30 
        time.sleep(1)
        motor_msg.drive_motors.dt_speed = 0
        time.sleep(1)
        motor_msg.drive_motors.dt_speed = -30
        time.sleep(1)
        motor_msg.drive_motors.dt_speed = 0

    def activate_crook(self):
        motor_msg = MotorCommand()
        motor_msg.dt_speed = -10
        self.elevator_timer_count += 1

        if self.elevator_timer_count >= 100: #1 second
            motor_msg.dt_speed = 0
            motor_msg.left_speed = -10
            motor_msg.right_speed = -10
        if self.elevator_timer_count >= 150: #.5 sec
            motor_msg.left_speed = 0
            motor_msg.right_speed = 0
            self.activate_elevator()




def main(args=None):
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

