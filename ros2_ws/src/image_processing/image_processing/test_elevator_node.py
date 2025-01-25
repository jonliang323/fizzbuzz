import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import MotorCommand
import keyboard

class ElevatorTestNode(Node): #testing elevator and duck
    def __init__(self):
        super().__init__('test_elevator_subscriber')
        self.clock = Clock()
        self.prev_time = self.clock.now()
        motor_msg = MotorCommand()
        self.i = 0
        self.elevator_state = 'open claws'
        self.elevator_timer_count = 0
        self.duck_timer_count = 0
        self.angle1_elev = 90
        self.angle2_claw = -40
        self.angle3_flap = 0
        self.elevator = False
        # self.angle4_duck = 0
        self.elevator_control = self.create_timer(0.5, self.elevator_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

    
    def elevator_callback(self):
        # print("callback triggered")
        motor_msg = MotorCommand()
        # dc = motor_msg.drive_motors
        # servo = motor_msg.actuate_motors
        motor_msg.actuate_motors.angle1_elev = self.angle1_elev
        motor_msg.actuate_motors.angle2_claw = self.angle2_claw
        motor_msg.actuate_motors.angle3_flap = self.angle3_flap
        # motor_msg.actuate_motors.angle4_duck = self.angle4_duck
        

        self.activate_elevator()
        self.motor_pub.publish(motor_msg)


        

    def activate_elevator(self):
        if self.elevator == False:
            # motor_msg = MotorCommand()

            # motor_msg.actuate_motors.angle1_elev = self.angle1_elev
            # motor_msg.actuate_motors.angle2_claw = self.angle2_claw
            # motor_msg.actuate_motors.angle3_flap = self.angle3_flap

            #elevator starts: angle1 = 90 (elevator up) angle2 = -40 (claws closed) angle3 = 0 (flap closed)
            # self.angle1_elev = 90
            # self.angle2_claw = -40
            # self.angle3_flap = -90
            # print(f'state: {self.elevator_state}')
            # print(f"State: {self.elevator_state}, Timer Count: {self.elevator_timer_count}")

            # if (self.clock.now() - self.prev_time).nanoseconds/1e9 > 2:
            #     self.i = self.i + 1
            #     self.prev_time = self.clock.now()

        

            if self.elevator_state == 'open claws':
            # if self.i == 0:
                self.angle1_elev = 90
                self.angle3_flap = -90 #flap open...if we want default flap to be closed, we can open it above when we set block_intake to be true
                self.angle2_claw = -45  #claws open
                #robot should move forward at this point. 

                self.elevator_timer_count += 1
                print(f'state: open claws ------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'elev move down'
                    print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move down':
            # elif self.i == 1:
                self.angle1_elev = -90  #elevator moves down
                self.angle3_flap = 0    #flap closes 
                self.angle2_claw = -45
                print(f'state: elev moving down------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'close claws'
                    print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            #return elevator to starting position
            elif self.elevator_state == 'close claws':
            # elif self.i == 2:
                self.angle1_elev = -90
                self.angle3_flap = 0 
                self.angle2_claw = -20  #claws close
                self.elevator_timer_count += 1
                print(f'state: claws close------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'elev move up'
                    print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move up':
            # elif self.i == 3:
                self.angle1_elev = 90  #elevator moves up while holding block
                self.angle3_flap = -90   #flap opens
                self.angle2_claw = -20
                self.elevator_timer_count += 1
                print(f'state: elev moving up------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'idle'
                    print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
            
            if self.elevator_state == 'idle':
            # elif self.i == 4:
                self.angle1_elev = 90  
                self.angle3_flap = 0   
                self.angle2_claw = -20
                print(f'state: idle ------angles: elev: {self.angle1_elev} claw: {self.angle2_claw} flap: {self.angle3_flap}')
                # print(f'start angles: elev: {self.angle1_elev}, claw: {self.angle2_claw}, flap: {self.angle3_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'open claws'
                    print(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
                self.elevator = True





def main(args=None):
    rclpy.init()
    node = ElevatorTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
  
