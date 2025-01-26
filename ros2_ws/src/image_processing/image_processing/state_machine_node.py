import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import board
import busio
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import EncoderCounts
from image_processing_interfaces.msg import MotorCommand
from std_msgs.msg import Bool

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.dT = 0.1 #seconds
        self.clock = Clock()

        self.FOV_XY = 480,680
        self.MAX_SPEED = 100
        self.NORM_SPEED = 50
        self.MAX_DELTA = self.MAX_SPEED - self.NORM_SPEED
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
        self.current_pos = (0,0)

        self.turn = True

        self.block_align_drive = False
        self.prev_error_turn = 0
        self.error_integralL_turn, self.error_integralR_turn= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn = 1,0,0
        self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn = 1,0,0

        # 360 scan variables
        self.scan_270_active = False
        self.scan = True
        self.detected_objects = []
        self.spin_speed = 50 #30 is placeholder 
        self.time_counter = 0

        #elevator variables
        self.block_intake = False
        self.first_sphere_grabbed = False
        self.red_counter = 0
        self.elevator_timer_count = 0
        self.elevator_state = 'idle'
        self.angle1_elev = 0
        self.angle2_claw = 0
        self.angle3_flap = 0
        self.angle4_duck = 0

        self.cv_sub = self.create_subscription(CubeTracking, "cube_location_info", self.cv_callback, 10)
        self.delta_encoder_sub = self.create_subscription(EncoderCounts, "delta_encoder_info", self.delta_encoder_callback, 10)
        #self.state_machine = self.create_timer(self.dT, self.state_machine_callback)
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
        self.get_logger().info(f'Requested callback')
        #enter here when scan is true, from callback recall initiate
        distances = msg.distances
        x_centers = msg.x_centers
        obj_type = msg.obj_types

        #obj has descriptors: distance, angle, x_center, type
        if distances != []:
            closest = self.find_closest_index(distances)
            closest_obj = {"distance":distances[closest], "angle":self.current_angle, "center":x_centers[closest], "type":self.CLASSES[obj_type[closest]]}
            self.detected_objects.append(closest_obj)
        #otherwise, detected objects is not changed
        self.scan = False


    def delta_encoder_callback(self, msg: EncoderCounts):
        self.delta_encoderL = msg.encoder1
        self.delta_encoderR = msg.encoder2
        # self.get_logger().info(f'delta_encoder: {self.delta_encoderL}, {self.delta_encoderR}')
        #only update angle here, after encoder deltas have been sent, to be read once per cycle
        self.update_angle_and_pos()
        # self.get_logger().info(f'current_angle: {self.current_angle}')
        # self.get_logger().info(f'current_position: {self.current_pos}')

    def state_machine_callback(self): #called every 0.5 seconds
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel
        # motor_msg = MotorCommand()
        # dc = motor_msg.drive_motors
        # servo = motor_msg.actuate_motors
        
        if self.scan_270_active:
            #spins full 270
            if self.turn_angle <= 270:# and self.time_counter < 20: #may want to outsource PID function to call anytime we turn
                #turns 270 degrees
                #scan is true
                if not self.scan and self.current_angle < self.turn_angle: #ccw turn, + angle
                    deltaL = 0#-self.spin_speed
                    deltaR = 0#self.spin_speed
                elif not self.scan: #self.current_angle matched self.turn_angle
                    self.scan = True
                    self.turn_angle += 90
            else: #scan is complete
                self.scan_270_active = False
                deltaL = 0
                deltaR = 0
                if len(self.detected_objects) > 0:
                    closest = self.find_closest_index([obj["distance"] for obj in self.detected_objects])
                    self.target = self.detected_objects[closest]
                else:
                    self.target = {"distance":None, "angle":0, "center":None, "type":None}
                print(f'\n\n scan complete, closest object is {self.target["type"]},\n{self.target["distance"]} away at an angle of {self.target["angle"]} \n\n\n\n')
                print(f'\n\n all detected_objects: {self.detected_objects} \n\n\n\n')
                #*** turn to face self.target["angle"] here:
                #*******
 
                # angle_diff = self.target["angle"] - self.current_angle
                
                # if angle_diff > 180:
                #     angle_diff -= 360
                # elif angle_diff < -180:
                #     angle_diff += 360
                
                # if abs(angle_diff) > 5:  #error of 5(?) degrees
                #     if angle_diff > 0:
                #         deltaL = -self.spin_speed
                #         deltaR = self.spin_speed
                #     else:
                #         deltaL = self.spin_speed
                #         deltaR = -self.spin_speed
                # else:
                #     # self.block_align_drive = True
                #     deltaL = 0
                #     deltaR = 0

            self.time_counter += 1
        
        if self.turn:
            gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
            gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)
            deltaL, deltaR = self.PID(self.turn_angle - self.current_angle, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
            self.get_logger().info(f'deltaL, deltaR: {deltaL} {deltaR}')

        # PID alignment, while still or driving; cube to align to should depend on recorded angle
        # if self.block_align_drive:
        #     #positive error -> turn left
        #     #negative error -> turn right
        #     cur_align_error = self.FOV_XY[0]//2 - self.target["center"]
        #     #capped integral terms
        #     self.error_integralL = max(min(self.error_integralL + cur_align_error*self.dT, self.MAX_DELTA/self.i_gainL), -self.MAX_DELTA/self.i_gainL)
        #     self.error_integralR = max(min(self.error_integralR + cur_align_error*self.dT, self.MAX_DELTA/self.i_gainR), -self.MAX_DELTA/self.i_gainR)
        #     error_deriv = (cur_align_error - self.prev_align_error)/self.dT

        #     if abs(self.error_integralL) == self.MAX_DELTA/self.i_gainL:
        #         print("Left integral saturated")
        #     if abs(self.error_integralR) == self.MAX_DELTA/self.i_gainR:
        #         print("Right integral saturated")
            
        #     #capped deltas
        #     deltaL = -max(min(self.p_gainL * cur_align_error + self.i_gainL * self.error_integralL + self.d_gainL * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)
        #     deltaR = max(min(self.p_gainR * cur_align_error + self.i_gainR * self.error_integralR + self.d_gainR * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)

            # #Drive until condition is met
            # if self.target["distance"] > 2: #to within 2 cm
            #     norm_speed = self.NORM_SPEED
            # else: #stop
            #     norm_speed = 0
            #     self.block_align_drive = False
            #     self.block_intake = True

            # self.prev_align_error = cur_align_error

        #TODO: block intake, elevator, bird, dumptruck
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

        #set motor speeds
        dc.left_speed = norm_speed + deltaL
        dc.right_speed = norm_speed + deltaR
        servo.angle1_elev = self.angle1_elev
        servo.angle2_claw = self.angle2_claw
        servo.angle3_flap = self.angle3_flap
        servo.angle4_duck = self.angle4_duck

        self.motor_pub.publish(motor_msg)

        scan_msg = Bool()
        scan_msg.data = self.scan
        self.scan_pub.publish(scan_msg)
    
    def find_closest_index(self, distances):
        #find closest distance index
        min = 0
        for i in range(1, len(distances)): #detected_objects is [type, distance, angle of view]
            if distances[i] < distances[min]:  
                min = i
        return min
    
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
            self.current_angle = self.modulate_angle(self.current_angle + round((dtheta*180/math.pi),2))

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
            print("Left integral saturated")
        if abs(error_integralR) == self.MAX_DELTA/i_gainR:
            print("Right integral saturated")
        
        #capped deltas
        deltaL = -max(min(p_gainL * cur_align_error + i_gainL * error_integralL + d_gainL * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)
        deltaR = max(min(p_gainR * cur_align_error + i_gainR * error_integralR + d_gainR * error_deriv, self.MAX_DELTA), -self.MAX_DELTA)

        return deltaL, deltaR


    def activate_elevator(self):
        motor_msg = MotorCommand()

        #elevator starts: angle1 = 90 (elevator up) angle2 = -25 (claws closed) angle3 = 0 (flap open)
        self.elevator_state == 'open claws'
        self.elevator_timer_count = 0
        

        if self.elevator_state == 'open claws':
            self.angle2_claw = -40  #claws open
            self.elevator_timer_count += 1

            if self.elevator_timer_count >= 4:  #wait 2 seconds
                self.elevator_state = 'elev move down'
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move down':
            self.angle1_elev = -90  #elevator moves down
            self.angle3_flap = 90   #flap closes

            self.elevator_timer_count += 1
            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'close claws'
                self.elevator_timer_count = 0

        #return elevator to starting position
        elif self.elevator_state == 'close claws':
            self.angle2_claw = -25  #claws close
            self.elevator_timer_count += 1
            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'elev move up'
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move up':
            self.angle1_elev = -90  #elevator moves up while holding block
            self.angle3_flap = 90   #flap opens, ready for next block
            self.elevator_timer_count += 1

            if self.elevator_timer_count >= 4:  
                self.elevator_state = 'idle'
                self.elevator_timer_count = 0
        
    #TODO: implement bird
    def activate_bird(self):
        #activate bird when red block comes
        motor_msg = MotorCommand()

        motor_msg.actuate_motors.angle4 = -90 #idk what the bird angles are
    
    #TODO: implement dumptruck
    def activate_dumptruck(self):
        motor_msg = MotorCommand()
        #some way to drive to purple wall
        motor_msg.drive_motors.left_speed = 30 #change this, maybe add something in DCCommand.msg like dt_speed and set to 0 for everything except when dt is activated
    

def main(args=None):
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

