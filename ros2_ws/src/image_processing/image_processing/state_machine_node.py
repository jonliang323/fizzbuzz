import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from image_processing_interfaces.msg import CubeTracking
from image_processing_interfaces.msg import MotorCommand

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_subscriber')
        self.clock = Clock()
        self.prev_time = self.clock.now()

        self.FOV_XY = 480,680
        self.MAX_SPEED = 100
        self.NORM_SPEED = 68
        self.MAX_DELTA = self.MAX_SPEED - self.NORM_SPEED

        self.block_align = False
        self.prev_align_error = 0
        self.error_integralL, self.error_integralR = 0,0
        #gains should result in critical damping, best response
        self.p_gainL, self.i_gainL, self.d_gainL = 1,0,0
        self.p_gainR, self.i_gainR, self.d_gainR = 1,0,0

        self.state_machine_sub = self.create_subscription(CubeTracking, "cube_location_info", self.state_machine_callback, 10)

        self.motor_pub = self.create_publisher(MotorCommand, "motor_command", 10)

    def state_machine_callback(self, msg: CubeTracking):
        cur_time = self.clock.now()
        dT = (self.clock.now() - self.prev_time).nanoseconds/1e9
        deltaL, deltaR = 0,0
        norm_speed = 0 #no decel

        #Drive until condition is met
        if self.drive_condition(msg):
            norm_speed = self.NORM_SPEED

        # PID alignment, while still or driving; cube to align to depends on msg.center_x
        if self.block_align:
            cur_align_error = self.FOV_XY[0]//2 - msg.center_x
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
        
        motor_msg = MotorCommand()
        motor_msg.left_speed = norm_speed + deltaL
        motor_msg.right_speed = norm_speed + deltaR

        self.prev_time = self.clock.now() #in case state_machine takes a bit to run
        self.motor_pub.publish(motor_msg)
        
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

