import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from icm42688 import ICM42688
import math
import board
import busio
import time
from image_processing_interfaces.msg import CubeTracking, EncoderCounts, MotorCommand, WallInfo, CVStates

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.dT = 0.01 #seconds
        self.clock = Clock()
        #self.timer = 0.0

        self.FOV_XY = 640,480
        self.FOCAL_X = 888.54513
        self.MAX_SPEED = 100
        self.NORM_SPEED = 30
        self.MAX_DELTA = 50
        self.WHEEL_RADIUS = (3.1875/2)*2.54 #cm
        self.BASE_RADIUS = (10.125/2)*2.54 #cm
        self.CLASSES = ['green','red','sphere']
        self.current_stack = (None,None)

        #cv variables
        #encoder variables
        self.delta_encoderL = 0
        self.delta_encoderR = 0
        self.ENCODER_RES = 640

        #heading, positioning variables
        self.current_angle = 0.0
        self.turn_angle = 40
        #self.current_pos = (0, 0)

        self.camera_startup = False

        # 360 scan variables
        self.scan_270_active = False
        self.scan_timer = 0
        #each of the following three depends on the one above it to be true
        self.wall_scan = False
        self.yolo_scan = False
        self.save_scan = False
        self.detected_objects = []
        self.detected_stacks = []
        self.detected_types = []
        self.detected_sizes = []
        self.stack_counter = 0
        self.error_deriv = 0

        #turning state, PID
        self.target_align = False
        self.prev_error_turn = 0
        self.error_integralL_turn, self.error_integralR_turn= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn = 1.8,0.01,0.1
        self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn = 1.8,0.01,0.1

        #turn drive state, PID
        self.target_drive = False
        self.prev_error_drive = 0
        self.error_integralL_drive, self.error_integralR_drive= 0,0
        #gains should result in critical damping, best response
        self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive = 0.1,0.01,0.1
        self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive = 0.1,0.01,0.1

        self.target = None
        # self.obj_types = None
        # self.sizes = None
        # self.x_centers = None
        # self.y_centers = None
        self.x_lefts = None
        self.y_tops = None
        self.X_ALIGN = 579
        self.Y_ALIGN = 360
        self.lost = False
        self.pixel_error = 0

        self.block_screen_ratio = 0
        self.wall_height_screen_ratio = 0
        self.orange_tape_ratio = 0
        self.orange_wall_found = False

        #elevator variables
        self.block_intake = True #True
        self.first_sphere_grabbed = True
        self.red_counter = 0
        self.elevator_timer_count = 0
        self.duck_timer_count = 0
        self.crook_timer_count = 0
        self.dumptruck_timer_count = 0
        self.elevator = False
        self.dumptruck = False
        self.duck = False
        self.crook = False
        self.elevator_state = 'open claws'
        self.angle1_duck = 70
        self.angle2_claw = -20
        self.angle3_elev = 90
        self.angle4_flap = 35
        self.intake_timer_count = 0
        self.moved = False
        self.dumptruck_state = 'dumping'
        self.dt_speed = 0


        self.competition_timer_count = 0

        self.cube_info_sub = self.create_subscription(CubeTracking, "cube_info", self.scan_block_callback, 10)
        self.wall_info_sub = self.create_subscription(WallInfo, "wall_info", self.scan_wall_callback, 10)
        self.delta_encoder_sub = self.create_subscription(EncoderCounts, "delta_encoder_info", self.delta_encoder_callback, 10)
        self.state_machine = self.create_timer(self.dT, self.state_machine_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)
        self.cv_pub = self.create_publisher(CVStates, "scan_blocks", 10) #tells cv if we want to scan blocks

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
        y_centers = msg.y_centers
        x_lefts = msg.x_lefts
        y_tops = msg.y_tops

        # self.get_logger().info(f'x left and y top: {x_lefts}, {y_tops}')
        #block_pixels = msg.block_pixels

        if len(obj_types) > 0:
            self.lost = False
            closest = self.find_closest_index(sizes)
            #two cases so far when we want to do something with the cv data
            if self.scan_270_active: #shouldn't this be if self.scan?
                #obj has descriptors: distance, angle, x_center, type
                rel_angle = math.atan((self.FOV_XY[0]/2 - x_centers[closest])/self.FOCAL_X)*180/math.pi
                closest_obj = {"size":sizes[closest], "angle":int(self.current_angle + rel_angle), "type":self.CLASSES[obj_types[closest]]}
                current_stack = self.find_stack(closest, closest_obj, x_centers, y_centers, sizes, obj_types)
                self.detected_objects.append(closest_obj)
                self.detected_stacks.append(current_stack)
                self.detected_types.append(closest_obj['type'])
                self.detected_sizes.append(closest_obj['size'])

                self.get_logger().info(f'closest stack in frame: {current_stack}')
                #otherwise, detected objects is not changed
                self.yolo_scan = False
                self.save_scan = False
            elif self.target_drive:
                pass
                # pass
                #self.block_screen_ratio = block_pixels/(self.FOV_XY[0]*self.FOV_XY[1]) #find block screen ratio
                #find, record x_center of closest block -> used in pid driving to align blocks with intake
                # self.x_lefts = x_lefts
                # self.y_tops = y_tops

                # self.get_logger().info(f'self x and y: {self.x_lefts},  {self.y_tops}')
                #keep on scanning, processing with yolo

            self.x_lefts = x_lefts
            self.y_tops = y_tops

            self.get_logger().info(f'self x and y: {self.x_lefts},  {self.y_tops}')
        else:
            self.lost = True
            self.pixel_error = -self.pixel_error*0.8
        # else:
        #     self.block_screen_ratio = 0

    def scan_wall_callback(self, msg: WallInfo):
        self.wall_height_screen_ratio = msg.avg_height/self.FOV_XY[1]
        self.orange_tape_ratio = msg.orange_tape_ratio
        self.height_range = msg.height_range

    def delta_encoder_callback(self, msg: EncoderCounts):
        self.delta_encoderR = msg.encoder1
        self.delta_encoderL = msg.encoder2
        #self.get_logger().info(f'{self.delta_encoderL, self.delta_encoderR}') #here
        #only update angle here, after encoder deltas have been sent, to be read once per cycle
        self.update_angle_and_pos()
        #self.get_logger().info(f'current_angle: {self.current_angle}')
        #self.get_logger().info(f'current_position: {self.current_pos}')

    #TODO detecting wall, staying away from wall
    #TODO identifying divider wall, moving towards it when using dumptruck
    #TODO keep track of the number of red blocks we have in our dumptruck to know if we can get any more
    #-> maybe we go to the center of the playing field after finding all of the stacks/red blocks
    #-> center of playing field determined by when wall_height_screen ratio is the same all the way around
    #-> and spin around until we see the orange wall, mark its angle, and then pid towards it/position
    #TODO spinning in place
    def state_machine_callback(self): #called every 0.01 seconds
        self.competition_timer_count += 1
        if self.competition_timer_count <= 13500:
            deltaL, deltaR = 0,0
            norm_speed = 0 #no decel

            if self.camera_startup:
                if self.competition_timer_count > 200:
                    self.get_logger().info('entered main seq')
                    self.camera_startup = False
                    self.scan_270_active = True
                    self.wall_scan = True
                    self.yolo_scan = True
                    self.save_scan = True

            if self.scan_270_active:
                #spins full 270
                if self.turn_angle <= 0: #320
                    #turns 270 degrees
                    #scan_blocks is initially true
                    self.get_logger().info(f'target: {self.turn_angle}, current: {self.current_angle}, angle diff: {self.turn_angle - self.current_angle}') #here
                    if not self.yolo_scan and abs(self.current_angle - self.turn_angle) > 10: #ccw turn, + angle

                        gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
                        gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)

                        angle_err = self.turn_angle - self.current_angle
                        deltaL, deltaR = self.PID(angle_err, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
                        self.prev_error_turn = angle_err

                    elif not self.yolo_scan: #just finished turning, next yolo scan
                        self.yolo_scan = True
                        self.save_scan = True

                    else:
                        deltaL = deltaR = 0
                        self.scan_timer +=1
                        if self.scan_timer >= 75: #.75 seconds
                            self.get_logger().info('exiting to next turn')
                            self.yolo_scan = False
                            self.turn_angle += 40
                            self.scan_timer = 0
                else: #scan is complete
                    #pass
                    deltaL = 0
                    deltaR = 0
                    #process list
                    if len(self.detected_objects) > 0:
                        closest = self.find_closest_index([obj["size"] for obj in self.detected_objects])
                        self.target = self.detected_objects[closest]
                        self.current_stack = self.detected_stacks[closest]
                        self.get_logger().info(f'detected stack: {self.current_stack}') #here
                        self.get_logger().info(f'\n\n scan complete, closest object is {self.target["type"]},\nat an angle of {self.target["angle"]}')
                        self.get_logger().info(f'\n\n{len(self.detected_objects)} detected_objects\n\n')
                        self.get_logger().info(f'These are our objects: {self.detected_objects}')
                        self.detected_objects = []
                        self.scan_270_active = False
                        # self.target_align = True
                        self.target_drive = True
                        self.yolo_scan = True #should be False
                        self.save_scan = True #should be False
                        self.prev_error_turn = 0
                    elif len(self.detected_objects) == 0 and self.stack_counter < 5:
                        #TODO: if no cubes found while there are still cubes to be found --> scan again
                        self.turn_angle = 0
                        self.yolo_scan = False #rotate first back to 0, then start scanning again
                        self.get_logger().info('found no objects')
                    else: #finished with stacks
                        self.get_logger().info('going to dump')
                        # self.activate_dumptruck()

            if self.target_align:
                self.get_logger().info(f'aligning....target angle: {self.target['angle']}, current angle: {self.current_angle}')
                if abs(self.target['angle'] - self.current_angle) > 10:
                # if abs(self.turn_angle - self.current_angle) > 10:
                    #*** turn to face self.target["angle"] here:
                    gainsL = (self.p_gainL_turn, self.i_gainL_turn, self.d_gainL_turn)
                    gainsR = (self.p_gainR_turn, self.i_gainR_turn, self.d_gainR_turn)

                    angle_error = (self.target['angle'] - self.current_angle + 180) % 360 - 180

                    #self.error_deriv = (angle_error - self.prev_error_turn)/self.dT
                    deltaL, deltaR = self.PID(angle_error, self.prev_error_turn, self.error_integralL_turn, self.error_integralR_turn, gainsL, gainsR)
                    #self.get_logger().info(f'deltas: {deltaL, deltaR}')
                    self.prev_error_turn = angle_error
                else:
                    #TODO constant scan for cube -> find % cube of screen -> have time delay? sparse scanning
                    deltaL = deltaR = 0
                    self.yolo_scan = True #save_scan is still False for target_drive
                    self.target_align = False
                    self.target_drive = True
                    self.prev_error_turn = 0

            # PID alignment, while still or driving; cube to align to should depend on recorded angle
            if self.target_drive:
                self.get_logger().info(f'closest stack overall: {self.current_stack}')
                type = self.current_stack[0]['type']
                b_index = self.find_closest_index(self.detected_sizes,types=self.detected_types, filter_type=type)
                target_y_top, target_x_left = self.y_tops[b_index], self.x_lefts[b_index]
                if (target_y_top < self.Y_ALIGN): #drive condition
                # if (self.timer < 2):
                    #self.timer += self.dT
                    gainsL = (self.p_gainL_drive, self.i_gainL_drive, self.d_gainL_drive)
                    gainsR = (self.p_gainR_drive, self.i_gainR_drive, self.d_gainR_drive)

                    if not self.lost:
                        self.pixel_error = self.X_ALIGN - target_x_left
                    self.get_logger().info(f'x,y: {target_x_left, target_y_top}, error {self.pixel_error}')

                    deltaL, deltaR = self.PID(self.pixel_error, self.prev_error_drive, self.error_integralL_drive, self.error_integralR_drive, gainsL, gainsR)
                    # deltaL = deltaR = 0
                    norm_speed = self.NORM_SPEED
                    self.prev_error_drive = self.pixel_error

                else:
                    self.get_logger().info(f'self.target_drive stopped')
                    norm_speed = 0
                    deltaL, deltaR = 0,0
                    self.yolo_scan = False
                    self.target_drive = False
                    # self.block_intake = True
                    self.prev_error_drive = 0

            if self.block_intake:
                queue = self.current_stack
                # self.get_logger().info(f'queue: {queue}')
                queue = [{"size": 0, "angle": 0, "type": "green"}, {"size": 0, "angle": 0, "type": 'red'}]
                # if queue[1] is not None:
                #     self.stack_counter += 1

                if self.first_sphere_grabbed == False:
                    # self.get_logger().info("grabbing first sphere")

                    # norm_speed, deltaL, deltaR, self.dt_speed = self.activate_dumptruck()
                    # self.get_logger().info(f"grabbing first sphere {self.dt_speed}")

                    # if self.dumptruck == True:
                    #     self.block_intake = False

                    norm_speed, self.dt_speed = self.activate_crook()
                    if self.crook == True:
                        self.first_sphere_grabbed = True


                else:
                    # self.get_logger().info("done with frist sphere, grabbing stack")


                    # deltaL = deltaR = 30


                    # deltaL = deltaR = 0 #go forwards a little here (ex. 1 sec) to have it move forward enough so that red block would be flush against flap and would knock stack over
                    factor = 2.5
                    if self.moved:
                        if queue[1] is not None and queue[1]["type"] == 'green':
                            #sequence here for red then green
                            if self.intake_timer_count < 300:
                                self.intake_timer_count +=1
                                self.activate_bird()
                            if self.intake_timer_count < 300 and self.intake_timer_count >= 400:
                                self.intake_timer_count +=1
                            if self.intake_timer_count >= 400 and self.intake_timer_count < 450:
                                self.angle4_flap = -90
                                self.intake_timer_count +=1
                            if self.intake_timer_count >= 450 and self.intake_timer_count < 500:
                                norm_speed = 30  #move forward some so green is in elevator shaft
                                self.intake_timer_count +=1
                            if self.intake_timer_count >= 500 and self.intake_timer_count < 1300:
                                norm_speed = 0
                                self.activate_elevator()
                            if self.intake_timer_count >= 1300:
                                self.intake_timer_count = 0
                                self.block_intake = False
                                # self.scan_270_active = True



                        elif queue[1] is not None and queue[1]["type"] == 'red':
                            self.get_logger().info(f"in stack queue: {queue[0]["type"]} {queue[1]["type"]}")
                            #seqeunce here for green then red

                            if self.intake_timer_count >= 0 and self.intake_timer_count < 25*factor:
                                norm_speed = 30 #move forward some so green is in elevator shaft
                                self.intake_timer_count +=1
                                self.get_logger().info("going to elevator activation")
                            if self.intake_timer_count >= 25*factor and self.intake_timer_count < 925: 
                                norm_speed = 0
                                self.activate_elevator()
                                self.intake_timer_count +=1

                            if self.intake_timer_count >= 925 and self.intake_timer_count < 975: 
                                norm_speed = 30 #move forward some so red is flush with flap
                                self.intake_timer_count+=1

                            if self.intake_timer_count >= 975*factor and self.intake_timer_count < 1275*factor:
                                norm_speed = 0
                                # deltaL = deltaR = 0 #stop
                                self.activate_bird()

                                self.intake_timer_count+=1


                            if self.intake_timer_count >= 1275*factor:
                                self.intake_timer_count = 0
                                self.block_intake = False
                                
                                # self.scan_270_active = True

                        elif queue[0]["type"] == 'green':
                            #sequence here for green only
                            if self.intake_timer_count < 25:
                                norm_speed = 30 #move forward some so green is in elevator shaft
                                self.intake_timer_count+=1
                            if self.intake_timer_count >= 25 and self.intake_timer_count < 825:
                                self.activate_elevator()
                                self.intake_timer_count +=1
                            if self.intake_timer_count >= 825:
                                self.intake_timer_count = 0
                                self.block_intake = False
                                # self.scan_270_active = True

                        elif queue[0]["type"] == 'red':
                            #sequence here for red only
                            if self.intake_timer_count < 300:
                                self.activate_bird()
                                self.intake_timer_count +=1
                            if self.intake_timer_count >= 300:
                                self.intake_timer_count = 0
                                self.block_intake = False
                                # self.scan_270_active = True
                    else:
                        if self.intake_timer_count < 75*factor:
                            if queue[0]['type'] == 'green':
                                self.angle4_flap = -90
                            self.intake_timer_count +=1

                        if self.intake_timer_count >= 75*factor and self.intake_timer_count < 125*factor:
                            self.get_logger().info('moving')
                            self.intake_timer_count +=1
                            norm_speed = 30

                        if self.intake_timer_count >= 125*factor and self.intake_timer_count < 175*factor:
                            self.get_logger().info('done moving')
                            norm_speed = 0
                            self.intake_timer_count += 1

                        if self.intake_timer_count >= 175*factor:
                            self.intake_timer_count = 0
                            self.moved = True


            #default: if too close to wall, this state pops up, overrides any state controls
            #TODO some maneuvering mechanism to get aligned with the wall
            # if self.wall_height_screen_ratio > 0.4:
            #     norm_speed = -self.NORM_SPEED
            #     deltaL, deltaR = 0,0


        else:

            #activate_dumptruck()
            norm_speed = 0
            deltaL = deltaR = 0
            self.angle1_duck = -70
            self.angle3_elev = 90
            self.angle2_claw = -40
            self.angle4_flap = -90

        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        servo = motor_msg.actuate_motors

        #set motor speeds
        # self.get_logger().info(f'speeds: {norm_speed, deltaL, deltaR}')
        dc.left_speed = int(norm_speed + deltaL)
        dc.right_speed = int(norm_speed + deltaR)
        dc.dt_speed = self.dt_speed
        servo.angle1_duck = self.angle1_duck
        servo.angle2_claw = self.angle2_claw
        servo.angle3_elev = self.angle3_elev
        servo.angle4_flap = self.angle4_flap

        scan_msg = CVStates()
        scan_msg.wall = self.wall_scan
        scan_msg.yolo = self.yolo_scan
        scan_msg.save = self.save_scan

        # self.get_logger().info(f'{self.yolo_scan}')

        self.cv_pub.publish(scan_msg)
        self.motor_pub.publish(motor_msg)



    def find_closest_index(self, sizes, types=None, filter_type=None):
        #find closest distance index (largest size)
        max_i = 0
        for i in range(1, len(sizes)):
            if filter_type and types[i] == filter_type and sizes[i] > sizes[max_i]:
                max_i = i
            elif sizes[i] > sizes[max_i]:
                max_i = i
        # self.get_logger().info(f'types max i: {types[max_i]}')
        if filter_type and types[max_i] != filter_type:
            self.get_logger().info(f'Error, color not found')
        return max_i

    def find_stack(self, index_of_closest, closest_obj, x_centers, y_centers, sizes, obj_types):
        #find stack of blocks
        stack_partner_index = None
        self.get_logger().info(f'x_centers: {x_centers}, index of closest: {index_of_closest}')
        for x in x_centers:
            diff = int(x - x_centers[index_of_closest])
            if diff < 50 and x_centers.index(x) != index_of_closest: #difference of 20 pixels
                stack_partner_index = x_centers.index(x)
                rel_angle = int(math.atan((self.FOV_XY[0]/2 - x_centers[stack_partner_index])/self.FOCAL_X)*180/math.pi)
                stack_partner = {"size":sizes[stack_partner_index], "angle":self.current_angle + rel_angle, "type":self.CLASSES[obj_types[stack_partner_index]]}
        if stack_partner_index == None:
            bottom = closest_obj
            top = None
        elif int(y_centers[stack_partner_index]) > int(y_centers[index_of_closest]):
            bottom = stack_partner
            top = closest_obj
        else:
            bottom = closest_obj
            top = stack_partner
        #closest_obj = {"size":sizes[closest], "angle":self.current_angle + rel_angle, "type":self.CLASSES[obj_types[closest]]}
        return (bottom, top)


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

    def find_orange_wall(self):
        if self.orange_tape_ratio > 0.75 and self.height_range < 10:
            deltaL= 0
            deltaR = 0
            self.orange_wall_found = True
        else:
            deltaL = 30
            deltaR= -30

        return deltaL, deltaR




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
        self.get_logger().info("in activate elevator")
        if self.elevator == False:
            self.get_logger().info("elevator if")
            down = 40

            if self.elevator_state == 'open claws':
            # if self.i == 0:
                self.angle3_elev = 90
                self.angle4_flap = -90 #flap open...if we want default flap to be closed, we can open it above when we set block_intake to be true
                self.angle2_claw = -20  #claws stay closed
                #TODO: robot should move forward at this point.


                self.elevator_timer_count += 1
                self.get_logger().info(f'state: open claws ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap} and timer = {self.elevator_timer_count}')

                if self.elevator_timer_count >= 100:  #wait 2 seconds
                    self.elevator_state = 'flap closes'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'flap closes':
            # elif self.i == 1:
                self.angle3_elev = 90
                self.angle4_flap = down   #flap closes
                self.angle2_claw = -20
                self.get_logger().info(f'state: elev moving down------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 100:
                    self.elevator_state = 'elev move down'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move down':
            # elif self.i == 1:
                self.angle3_elev = -70  #elevator moves down
                self.angle4_flap = down   #flap stays closed
                self.angle2_claw = -45  #claws open at the same time
                self.get_logger().info(f'state: elev moving down------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 100:
                    self.elevator_state = 'close claws'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            #return elevator to starting position
            elif self.elevator_state == 'close claws':
            # elif self.i == 2:
                self.angle3_elev = -70
                self.angle4_flap = down
                self.angle2_claw = -20  #claws close
                self.elevator_timer_count += 1
                self.get_logger().info(f'state: claws close------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 100:
                    self.elevator_state = 'elev move up'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move up':
            # elif self.i == 3:
                self.angle3_elev = 90  #elevator moves up while holding block
                self.angle4_flap = down#-90   #flap opens
                self.angle2_claw = -20
                self.elevator_timer_count += 1
                self.get_logger().info(f'state: elev moving up------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 100:
                    self.elevator_state = 'idle'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            if self.elevator_state == 'idle':
            # elif self.i == 4:
                self.angle3_elev = 90
                self.angle4_flap = down
                self.angle2_claw = -20
                self.get_logger().info(f'state: idle ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'open claws'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
                self.elevator = True



    #TODO: implement bird
    def activate_bird(self):
        motor_msg = MotorCommand()
        # self.get_logger().info("activating duck")
        if self.duck == False:
            self.get_logger().info("entering duck if")

            self.angle1_duck = -80 #idk what the bird angles are
            self.duck_timer_count+=1

            if self.duck_timer_count >= 200:
                self.get_logger().info("going back")
                self.angle1_duck = 70
                self.duck = True


    def activate_dumptruck(self):
        self.get_logger().info("activate dumptruck")
        if self.dumptruck == False:
            #some way to drive to orange wall
            if self.dumptruck_state == 'turning':
                if self.orange_wall_found == False:
                    deltaL, deltaR = self.find_orange_wall()
                    norm_speed = 0
                    dt_speed = 0

                else:
                    norm_speed = 0
                    deltaL = deltaR = 0
                    dt_speed = 0
                    self.dumptruck_state = 'driving'
                    self.dumptruck_timer_count = 0

            if self.dumptruck_state == 'driving':
                dt_speed = 0
                if self.wall_height_screen_ratio < 0.7: #0.7 is placeholder
                    norm_speed = 30
                    deltaL = deltaR = 0

                else:
                    if self.dumptruck_timer_count < 50: #0.5 second:
                        norm_speed = 0
                        deltaL = -30
                        deltaR = 30
                        dt_speed = 0
                        self.dumptruck_timer_count += 1
                    else:
                        norm_speed = 0
                        deltaL = deltaR = 0
                        dt_speed = 0
                        self.dumptruck_state = 'dumping'
                        self.dumptruck_timer_count = 0


            if self.dumptruck_state == 'dumping':
                # self.get_logger().info("enter dump state")
                norm_speed = 0
                deltaL = deltaR = 0
                if self.dumptruck_timer_count < 200: #2 second
                    self.get_logger().info("in first stage")
                    # dt_speed = -50
                    norm_speed = 50
                    self.dumptruck_timer_count += 1
                if self.dumptruck_timer_count >= 200 and self.dumptruck_timer_count < 300: #1 second
                    self.get_logger().info("in second stage")
                    dt_speed = 0
                    self.dumptruck_timer_count += 1
                if self.dumptruck_timer_count >= 300 and self.dumptruck_timer_count < 500: #2 second
                    self.get_logger().info("in third stage")
                    dt_speed = 50
                    self.dumptruck_timer_count += 1
                if self.dumptruck_timer_count >= 500:
                    self.get_logger().info("in fourth stage")
                    dt_speed = 0
                    self.dumptruck = True

        return norm_speed, deltaL, deltaR, dt_speed



    def activate_crook(self):
        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        if self.crook == False:
            self.get_logger().info("activating crook")
            if self.crook_timer_count < 200:
                dt_speed = -50
                norm_speed = 0
                self.elevator_timer_count += 1
            if self.crook_timer_count >= 200 and self.crook_timer_count < 250: #1 second
                dt_speed = 0
                norm_speed = -10
                self.elevator_timer_count += 1
            if self.crook_timer_count >= 250: #.5 sec
                norm_speed=0
                dt_speed = 0
                self.activate_elevator() #will move forward in function
                self.first_sphere_grabbed = True
                self.crook = True

        return norm_speed, dt_speed





def main(args=None):
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
