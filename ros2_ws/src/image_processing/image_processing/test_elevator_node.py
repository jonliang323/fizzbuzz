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
        self.i = 0
        self.elevator_state = 'idle'
        self.elevator_timer_count = 0
        self.duck_timer_count = 0
        self.angle1_elev = 0
        self.angle2_claw = 0
        self.angle3_flap = 0
        self.angle4_duck = 0
        self.elevator_control = self.create_timer(0.5, self.elevator_callback)
        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

    
    def elevator_callback(self):
        motor_msg = MotorCommand()
        dc = motor_msg.drive_motors
        servo = motor_msg.actuate_motors
        servo.angle1_elev = self.angle1_elev
        servo.angle2_claw = self.angle2_claw
        servo.angle3_flap = self.angle3_flap
        servo.angle4_duck = self.angle4_duck

        self.activate_elevator()

        self.motor_pub.publish(motor_msg)

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



def main(args=None):
    rclpy.init()
    node = ElevatorTestNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check for keyboard input
            if keyboard.is_pressed('e'):
                node.get_logger().info("Elevator activated!")
                node.activate_elevator()

            elif keyboard.is_pressed('q'):
                node.get_logger().info("Exiting Elevator Tester.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
  
