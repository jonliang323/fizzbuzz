#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Point
from interfaces.msg import Detect, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

BASE_D = 269.0

class State(Enum):
    SEARCH = 0
    APPROACH = 1
    COLLECT = 2

class Object(Enum):
    GREEN = 0
    RED = 1
    SPHERE = 2

class PlanNode(Node):
    def __init__(self):
        super().__init__('plan')

        # computer vision
        self.load_detect = True
        self.detect = Detect()
        self.create_subscription(Detect, 'detect', self.detect_callback, 10)

        # move wheels
        self.moving = False
        self.create_subscription(Bool, 'moving', self.moving_callback, 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)

        # bucket dc motor
        self.bucket_pub  = self.create_publisher(Float32, 'bucket', 10)

        # servos
        self.duck_pub = self.create_publisher(Float32, 'duck', 10)
        self.claw_pub = self.create_publisher(Float32, 'claw', 10)
        self.elevator_pub = self.create_publisher(Float32, 'elevator', 10)
        self.flap_pub = self.create_publisher(Float32, 'flap', 10)

        # initialize servo position defaults
        self.drop_duck()
        self.close_claw()
        self.lift_elevator()
        self.close_flap()

        # other runtime variables
        self.has_sphere = False
        self.bottom_red = False # arbitrary (gets set later)

        self.get_logger().info("Plan node has started")

    #################### helper vision functions ########################
    # waits until new detect data from cv
    def wait_detect(self):
        # maybe wait because of cv pipeline delay
        time.sleep(1) #TODO
        self.load_detect = True
        while self.load_detect:
            time.sleep(0.1)
    
    # makes sure detect data is good
    # TODO probably should be in vision actually
    def valid_detect(self):
        pass # TODO
    
    ###################### callback functions ##########################
    def detect_callback(self, msg):
        if self.load_detect:
            self.detect = msg
            self.load_detect = False

    def moving_callback(self, msg):
        self.moving = msg.data
        self.get_logger().info(str(self.moving))

    ################## helper publish functions ######################
    # left and right are coefficients
    def pub_path(self, left, right, distance):
        msg = Path()
        msg.left = float(left)
        msg.right = float(right)
        msg.distance = float(distance)
        self.path_pub.publish(msg)

    def wait_path(self):
        while self.moving:
            time.sleep(0.1)
    
    def pub_bucket(self, dir):
        msg = Bool()
        msg.data = dir
        self.bucket_pub.publish(msg)

    def pub_duck(self, position):
        msg = Float32()
        msg.data = position
        self.duck_pub.publish(position)
    
    def pub_claw(self, position):
        msg = Float32()
        msg.data = position
        self.duck_pub.publish(position)
    
    def pub_elevator(self, position):
        msg = Float32()
        msg.data = position
        self.duck_pub.publish(position)
    
    def pub_flap(self, position):
        msg = Float32()
        msg.data = position
        self.duck_pub.publish(position)

    ################## helper time compensated move functions ######################
    def lift_bucket(self):
        self.pub_bucket(True)
        time.sleep(3) #TODO

    def drop_bucket(self):
        self.pub_bucket(False)
        time.sleep(3) #TODO
    
    def lift_duck(self):
        self.pub_duck(-80) #TODO
        time.sleep(1.5) #TODO

    def drop_duck(self):
        self.pub_duck(70) #TODO
        time.sleep(1.5) #TODO

    def open_claw(self):
        self.pub_claw(-45) #TODO
        time.sleep(0.5) #TODO

    def close_claw(self):
        self.pub_claw(-20) #TODO
        time.sleep(0.5) #TODO

    def lift_elevator(self):
        self.pub_elevator(90) #TODO
        time.sleep(1.0) #TODO

    def drop_elevator(self):
        self.pub_elevator(-70) #TODO
        time.sleep(1.0) #TODO

    def open_flap(self):
        self.pub_flap(-90) #TODO
        time.sleep(1.0) #TODO

    def close_flap(self):
        self.pub_flap(40) #TODO
        time.sleep(1.0) #TODO

    ####################### approach drive functions ###############################
    def drive_closest(self, object):
        red = self.detect.red
        green = self.detect.green
        sphere = self.detect.sphere

        if green.y == float('inf'):
            return
        
        dx = green.x - 2.9
        dy = green.y - 0.5

        self.drive_to(dx, dy, 1)

    # drives to x, y relative coordinates
    # percent of path 0-1
    def drive_to(self, x, y, percent):
        path = Path()

        if x == 0:
            path.left = 1
            path.right = 1
            path.distance = y*percent
        else:
            r = (y**2 + x**2)/(2*x)
            theta = math.atan2(y, abs(r-x))

            b_in = BASE_D/25.4
            z = b_in/(2*r)

            # strange constant, but affects turn sharpness
            z /= 2

            path.left = 1 + z
            path.right = 1 - z
            
            path.distance = abs(r*theta)*percent
        
        self.path_pub.publish(path)
        self.wait_path()
        self.get_logger().info(f"Path {path.left:.3f}, {path.right:.3f}, {path.distance:.3f}")

    ####################### main program functions ##########################

    # stacks after object inside chute
    def stack(self):
        self.open_claw()
        self.drop_elevator()
        self.close_claw()
        
        # drive backward incase flap is obstructed
        self.pub_path(-1, -1, 3) #TODO maybe change
        self.wait_path()

        self.close_flap()

        # optional reopen and close claw for better grip
        # self.open_claw()
        # self.close_claw()

        self.lift_elevator()

    def collect_green(self):
        self.open_flap()

        # drive forward
        self.pub_path(1, 1, 10) #TODO
        self.wait_path() #TODO

        self.stack()

    def collect_red(self):
        # drive to block
        self.pub_path(1, 1, 10) #TODO
        self.wait_path()

        self.lift_duck()
        self.drop_duck()


    def collect_sphere(self):
        self.drop_bucket()

        # knock off ball (drive backward)
        self.pub_path(-1, -1, 5) #TODO
        self.wait_path()

        self.open_flap()

        # drive to ball
        self.pub_path(1, 1, 10) #TODO
        self.wait_path()

        # stack sequence
        self.stack()

    def collect_all(self):
        # collect sphere
        if not self.has_sphere:
            self.has_sphere = True

            self.collect_sphere()
        
        # collect in proper order
        if self.bottom_red:
            self.collect_red()
            self.collect_green()
        else:
            self.collect_green()
            self.collect_red()

    def approach():
        pass

    def search():
        pass

    # main loop
    def loop(self):
        # TODO add drop off logic at the end either timer or loop
        self.search()
        self.approach()
        self.collect_all()

def main(args=None):
    rclpy.init(args=args)
    node = PlanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down plan node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()