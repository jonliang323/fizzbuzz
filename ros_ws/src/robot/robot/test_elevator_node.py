import keyboard
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from robot.msg import CubeTracking, MotorCommand


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
        self.angle3_elev = 90
        self.angle2_claw = -40
        self.angle4_flap = -70 #45

        self.elevator = True
        # self.angle4_duck = 0
        self.elevator_control = self.create_timer(0.5, self.elevator_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

    
    def elevator_callback(self):
        # self.get_logger().info("callback triggered")
        motor_msg = MotorCommand()
        # dc = motor_msg.drive_motors
        # servo = motor_msg.actuate_motors
        motor_msg.actuate_motors.angle3_elev = self.angle3_elev
        motor_msg.actuate_motors.angle2_claw = self.angle2_claw
        motor_msg.actuate_motors.angle4_flap = self.angle4_flap
        # motor_msg.actuate_motors.angle4_duck = self.angle4_duck
        

        # self.activate_elevator()
        self.motor_pub.publish(motor_msg)


        

    def activate_elevator(self):
        if self.elevator == True:      

            if self.elevator_state == 'open claws':
            # if self.i == 0:
                self.angle3_elev = 90
                self.angle4_flap = -70 #flap open...if we want default flap to be closed, we can open it above when we set block_intake to be true
                self.angle2_claw = -40  #claws stay closed
                #TODO: robot should move forward at this point. 


                self.elevator_timer_count += 1
                self.get_logger().info(f'state: open claws ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'flap closes'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'flap closes':
            # elif self.i == 1:
                self.angle3_elev = 90  
                self.angle4_flap = -70    #flap closes 
                self.angle2_claw = -1
                self.get_logger().info(f'state: flap closes------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'elev move down'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move down':
            # elif self.i == 1:
                self.angle3_elev = -70  #elevator moves down
                self.angle4_flap = -70    #flap stays closed
                self.angle2_claw = -45  #claws open at the same time
                self.get_logger().info(f'state: elev moving down------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                self.elevator_timer_count += 1
                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'close claws'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            #return elevator to starting position
            elif self.elevator_state == 'close claws':
            # elif self.i == 2:
                self.angle3_elev = -70
                self.angle4_flap = -70 
                self.angle2_claw = -1  #claws close
                self.elevator_timer_count += 1
                self.get_logger().info(f'state: claws close------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'elev move up'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0

            elif self.elevator_state == 'elev move up':
            # elif self.i == 3:
                self.angle3_elev = 90  #elevator moves up while holding block
                self.angle4_flap = -70   #flap opens
                self.angle2_claw = -1
                self.elevator_timer_count += 1
                self.get_logger().info(f'state: elev moving up------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 4:  
                    self.elevator_state = 'idle'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
            
            if self.elevator_state == 'idle':
            # elif self.i == 4:
                self.angle3_elev = 90  
                self.angle4_flap = -70   
                self.angle2_claw = -1
                self.get_logger().info(f'state: idle ------angles: elev: {self.angle3_elev} claw: {self.angle2_claw} flap: {self.angle4_flap}')

                if self.elevator_timer_count >= 4:  #wait 2 seconds
                    self.elevator_state = 'open claws'
                    # self.get_logger().info(f'state going to: {self.elevator_state}')
                    self.elevator_timer_count = 0
                self.elevator = False
                self.do_next_thing = True




def main(args=None):
    rclpy.init()
    node = ElevatorTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
  
