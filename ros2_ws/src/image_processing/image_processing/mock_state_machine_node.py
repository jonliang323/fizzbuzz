import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import board
import busio
import time
from image_processing_interfaces.msg import CubeTracking, EncoderCounts, MotorCommand, WallInfo
from std_msgs.msg import Bool

class MockStateMachineNode(Node):
    def __init__(self):
        super().__init__('mock_state_machine')
        self.dT = 0.01 #seconds
        self.clock = Clock()

        self.FOV_XY = 640,480
        self.FOCAL_X = 888.54513
        self.MAX_SPEED = 100
        self.NORM_SPEED = 0#30
        self.MAX_DELTA = 50
        self.WHEEL_RADIUS = (3.1875/2)*2.54 #cm
        self.BASE_RADIUS = (10.125/2)*2.54 #cm
        self.CLASSES = ['green','red','sphere']
        self.current_stack = (0,0)

        #cv variables
        #encoder variables
        self.delta_encoderL = 0
        self.delta_encoderR = 0
        self.ENCODER_RES = 640

        #heading, positioning variables
        self.current_angle = 0.0
        self.turn_angle = 90
        #self.current_pos = (0, 0)

        # 360 scan variables
        self.scan_270_active = False
        self.scan_blocks = False
        self.detected_objects = []
        self.spin_speed = 50 #30 is placeholder
        self.stack_counter = 0
        self.error_deriv = 0

        #turning state, PID
        self.target_align = True
        self.prev_error_turn = 0
        self.error_integralL_turn, self.error_integralR_turn= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn = 0.9,0.01,0.1
        self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn = 0.9,0.01,0.1

        #turn drive state, PID
        self.target_drive = False
        self.prev_error_drive = 0
        self.error_integralL_drive, self.error_integralR_drive= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive = 0.5,0.01,0.1
        self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive = 0.5,0.01,0.1

        self.block_screen_ratio = 0
        self.wall_height_screen_ratio = 0
        self.orange_tape_ratio = 0

        #elevator variables
        self.block_intake = False
        self.first_sphere_grabbed = False
        self.red_counter = 0
        self.elevator_timer_count = 0
        self.elevator = False
        self.dumptruck = False
        self.duck = False
        self.crook = False
        self.elevator_state = 'open claws'
        self.angle1_duck = 0
        self.angle2_claw = 0
        self.angle3_elev = 0
        self.angle4_flap = 0


        self.count = 0
        # self.angles = [0, -45, 45, 135]
        self.angles = [45, 90, 180, 270, 360]
        self.timer = 0.0
        # self.times = [1.5, 5, 3, 4]
        self.times = [1, 1, 1, 1, 1]
        #self.global_timer = 0




        self.cube_info_sub = self.create_subscription(CubeTracking, "cube_info", self.scan_block_callback, 10)
        self.wall_info_sub = self.create_subscription(WallInfo, "wall_info", self.scan_wall_callback, 10)
        self.delta_encoder_sub = self.create_subscription(EncoderCounts, "delta_encoder_info", self.delta_encoder_callback, 10)
        self.state_machine = self.create_timer(self.dT, self.state_machine_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)
        self.cv_pub = self.create_publisher(Bool, "scan_blocks", 10) #tells cv if we want to scan blocks

        #imu initialization
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

        while not spi.try_lock():
            pass

        spi.configure(baudrate=5000000)

        self.imu = ICM42688(spi)
        self.imu.begin()
    
    def scan_block_callback(self, msg: CubeTracking):
        self.get_logger().info(f'Requested cube scan')
        #enter here when scan_blocks is true
        obj_types = msg.obj_types
        sizes = msg.sizes
        x_centers = msg.x_centers
        block_pixels = msg.block_pixels

        #two cases when we want to scan the blocks (run YOLO classification)
        if self.scan_270_active: #shouldn't this be if self.scan?
            #obj has descriptors: distance, angle, x_center, type
            if obj_types != []:
                closest = self.find_closest_index(sizes)
                rel_angle = int(math.atan((self.FOV_XY[0]/2 - x_centers[closest])/self.FOCAL_X)*180/math.pi)
                closest_obj = {"size":sizes[closest], "angle":self.current_angle + rel_angle, "type":self.CLASSES[obj_types[closest]]}
                self.current_stack = self.find_stack(closest, closest_obj)
                self.detected_objects.append(closest_obj)
            #otherwise, detected objects is not changed
        elif self.target_drive:
            #find block screen ratio
            if obj_types != []:
                self.block_screen_ratio = block_pixels/(self.FOV_XY[0]*self.FOV_XY[1])
            else:
                self.block_screen_ratio = 0
        self.scan_blocks = False

    def scan_wall_callback(self, msg: WallInfo):
        self.wall_height_screen_ratio = msg.avg_height/self.FOV_XY[1]
        self.orange_tape_ratio = msg.orange_tape_ratio

    def delta_encoder_callback(self, msg: EncoderCounts):
        self.delta_encoderR = msg.encoder1
        self.delta_encoderL = msg.encoder2
        #self.get_logger().info(f'{self.delta_encoderL, self.delta_encoderR}')
        #only update angle here, after encoder deltas have been sent, to be read once per cycle
        self.update_angle_and_pos()
        self.get_logger().info(f'current_angle: {self.current_angle}')
        #self.get_logger().info(f'current_position: {self.current_pos}')

    #TODO detecting wall, staying away from wall
    #TODO identifying divider wall, moving towards it when using dumptruck
    #TODO keep track of the number of red blocks we have in our dumptruck to know if we can get any more
    #-> maybe we go to the center of the playing field after finding all of the stacks/red blocks
    #-> center of playing field determined by when wall_height_screen ratio is the same all the way around
    #-> and spin around until we see the orange wall, mark its angle, and then pid towards it/position
    #TODO spinning in place
    def state_machine_callback(self): #called every 0.01 seconds
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel
        #self.global_timer += self.dT

        if self.target_align and self.count < 5:
            if abs(self.angles[self.count] - self.current_angle) > 6:
                #*** turn to face self.target["angle"] here: 
                gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
                gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)

                # angle_error = self.target['angle'] - self.current_angle
                angle_error = self.angles[self.count] - self.current_angle
                if angle_error > 180:
                    angle_error -= 360
                elif angle_error < -180:
                    angle_error += 360

                #self.error_deriv = (angle_error - self.prev_error_turn)/self.dT
                deltaL, deltaR = self.PID(angle_error, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
                #self.get_logger().info(f'deltas: {deltaL, deltaR}')
                self.prev_error_turn = angle_error

            else:
                #TODO constant scan for cube -> find % cube of screen -> have time delay? sparse scanning
                self.timer = 0
                deltaL = deltaR = 0
                self.target_align = False
                self.target_drive = True
                self.prev_error_turn = 0

        # PID alignment, while still or driving; cube to align to should depend on recorded angle
        if self.target_drive:
            if (self.timer < self.times[self.count]):
                self.timer += self.dT
                gainsL = (self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive)
                gainsR = (self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive)

                # angle_err = self.target['angle'] - self.current_angle
                angle_error = self.angles[self.count] - self.current_angle
                # angle_error = 10
                if angle_error > 180:
                    angle_error -= 360
                elif angle_error < -180:
                    angle_error += 360
                deltaL, deltaR = self.PID(angle_error, self.prev_error_drive, self.error_integralL_drive, self.error_integralR_drive, gainsL, gainsR)
                # deltaL = deltaR = 0
                norm_speed = self.NORM_SPEED

                self.prev_errpr_drive = angle_error
            
            else:
                self.count+=1
                self.get_logger().info(f'self.target_drive stopped')
                norm_speed = 0
                self.target_drive = False
                self.target_align = True
                self.prev_error_drive = 0
        
        # if (self.timer < self.times[self.count]):
        #     self.activate_elevator()

        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        servo = motor_msg.actuate_motors

        # self.get_logger().info(f"lW: {-norm_speed + deltaL}, rW: {-norm_speed + deltaR}")

        #set motor speeds
        dc.left_speed = int(norm_speed + deltaL)
        dc.right_speed = int(norm_speed + deltaR)
        #dc.dt_speed = 30
        servo.angle1_duck = self.angle1_duck
        servo.angle2_claw = self.angle2_claw
        servo.angle3_elev = self.angle3_elev
        servo.angle4_flap = self.angle4_flap

        self.motor_pub.publish(motor_msg)

    
    def update_angle_and_pos(self):
        #should be positive or negative
        xL = self.delta_encoderL/self.ENCODER_RES*2*math.pi*self.WHEEL_RADIUS
        xR = self.delta_encoderR/self.ENCODER_RES*2*math.pi*self.WHEEL_RADIUS
        dtheta = (xR-xL)/(2*self.BASE_RADIUS) #rad

        # pos diff calculated before angle update
        #_primes are in rotated reference frame
        # if dtheta != 0:  
        #     dy_prime = (xL/dtheta + self.BASE_RADIUS)*math.sin(dtheta)
        #     dx_prime = -dy_prime*math.tan(dtheta)
        #     #rotating to inertial frame given current angle (before update)
        #     theta = self.current_angle*math.pi/180
        #     #rotation matrix abt z, applied: [cos -sin; sin cos] * [dx_prime; dy_prime] = [dx; dy]
        #     #-theta is used to rotate back to inertial frame, unsimplified for readability
        #     dx = math.cos(-theta)*dx_prime - math.sin(-theta)*dy_prime
        #     dy = math.sin(-theta)*dx_prime + math.cos(-theta)*dy_prime

        #     self.current_pos = (self.current_pos[0] - dx*(29/25), self.current_pos[1] + dy*(29/25))
        self.current_angle = self.modulate_angle(self.current_angle + round((dtheta*180/math.pi),2)) #CAN BE IN ABOVE IF STATEMENT

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
        if self.elevator_state == 'open claws':
        # if self.i == 0:
            self.angle3_elev = 90
            self.angle4_flap = -90 #flap open...if we want default flap to be closed, we can open it above when we set block_intake to be true
            self.angle2_claw = -45  #claws open
            #robot should move forward at this point. 

            self.elevator_timer_count += self.dT
            # print(f'state: open claws ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

            if self.elevator_timer_count >= 2:  #wait 2 seconds
                self.elevator_state = 'elev move down'
                # print(f'state going to: {self.elevator_state}')
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move down':
        # elif self.i == 1:
            self.angle3_elev = -90  #elevator moves down
            self.angle4_flap = 0    #flap closes 
            self.angle2_claw = -45
            # print(f'state: elev moving down------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

            self.elevator_timer_count += self.dT
            if self.elevator_timer_count >= 2:  
                self.elevator_state = 'close claws'
                # print(f'state going to: {self.elevator_state}')
                self.elevator_timer_count = 0

        #return elevator to starting position
        elif self.elevator_state == 'close claws':
        # elif self.i == 2:
            self.angle3_elev = -90
            self.angle4_flap = 0 
            self.angle2_claw = -20  #claws close
            self.elevator_timer_count += self.dT
            # print(f'state: claws close------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

            if self.elevator_timer_count >= 2:  
                self.elevator_state = 'elev move up'
                # print(f'state going to: {self.elevator_state}')
                self.elevator_timer_count = 0

        elif self.elevator_state == 'elev move up':
        # elif self.i == 3:
            self.angle3_elev = 90  #elevator moves up while holding block
            self.angle4_flap = -90   #flap opens
            self.angle2_claw = -20
            self.elevator_timer_count += self.dT
            # print(f'state: elev moving up------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

            if self.elevator_timer_count >= 2:  
                self.elevator_state = 'idle'
                # print(f'state going to: {self.elevator_state}')
                self.elevator_timer_count = 0
        
        if self.elevator_state == 'idle':
        # elif self.i == 4:
            self.angle3_elev = 90  
            self.angle4_flap = 0   
            self.angle2_claw = -20
            # print(f'state: idle ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

            if self.elevator_timer_count >= 4:  #wait 2 seconds
                self.elevator_state = 'open claws'
                # print(f'state going to: {self.elevator_state}')
                self.elevator_timer_count = 0






def main(args=None):
    rclpy.init()
    node = MockStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

