#!/usr/bin/env python3

import sys

import threading
import math
import time
from enum import Enum

import rclpy
from interfaces.action import Start
from interfaces.msg import Detect, Path
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Empty

BASE_D = 269.0

class Color(Enum):
    GREEN = 0
    RED = 1

class PlanNode(Node):
    def __init__(self):
        super().__init__('plan')

        # start action server
        self.action_server = ActionServer(self, Start, 'start', self.start_callback)

        self.end_timer = self.create_timer(1, self.check_time)

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
        self.open_flap()

        # other runtime variables
        self.has_sphere = True
        self.bottom_color = Color.GREEN # set later
        self.startTime = 0
        self.started = False

        self.get_logger().info("Plan node has started")

    #################### helper vision functions ########################
    # waits until new detect data from cv
    def wait_detect(self):
        # wait because of cv pipeline delay
        time.sleep(2.7)
        self.load_detect = True
        while self.load_detect:
            time.sleep(0.1)
    
    # returns boolean if valid block visible
    def block_visible(self):
        return self.detect.red.height != 0 or self.detect.green.height != 0
    
    # returns color of closest block
    # requires block to exist
    def closest_block(self):
        if self.detect.green.y >= self.detect.red.y:
            self.get_logger().info(f"Detected green {self.detect.green.y}")
            return Color.GREEN
        
        self.get_logger().info("Detected red")
        return Color.RED
    
    # calculates x, y relative coordinates of closest block in inches
    def block_coordinates(self, color):
        box = None
        if color == Color.GREEN:
            box = self.detect.green
        else:
            box = self.detect.red

        VFOV = 30*(math.pi/180)
        HFOV = 1.04*VFOV*(640/480)
        BLOCK_SIZE = 2 # inches

        # calculate y
        y = 480/(max(box.height, box.width)*math.tan(VFOV/2))

        # calculate x
        theta = box.x * (HFOV/640)
        x = float(BLOCK_SIZE*y*math.tan(theta)/2)

        # account for blocks to the side looking too close
        scalar = 1 + 0.04 * abs(x) / 5.8
        x *= scalar
        y *= scalar

        # account for camera position offset
        x -= 2.9
        y -= 0.5

        self.get_logger().info(f'x {x} y {y}')

        return (x,y)

    ###################### callback functions ##########################
    def detect_callback(self, msg):
        if self.load_detect:
            self.detect = msg
            self.load_detect = False

    def moving_callback(self, msg):
        self.moving = msg.data

    def check_time(self):
        if self.started and (time.time() - self.startTime) > 135:
            self.get_logger().info("Time up, exiting")
            self.open_claw()
            time.sleep(2)
            sys.exit(0)

    ################## helper publish functions ######################
    # left and right are coefficients
    def pub_path(self, left, right, distance):
        msg = Path()
        msg.left = float(left)
        msg.right = float(right)
        msg.distance = float(distance)
        self.path_pub.publish(msg)

    def wait_path(self):
        time.sleep(0.5)
        while self.moving:
            time.sleep(0.1)
    
    def pub_bucket(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.bucket_pub.publish(msg)

    def pub_duck(self, position):
        msg = Float32()
        msg.data = float(position)
        self.duck_pub.publish(msg)
    
    def pub_claw(self, position):
        msg = Float32()
        msg.data = float(position)
        self.claw_pub.publish(msg)
    
    def pub_elevator(self, position):
        msg = Float32()
        msg.data = float(position)
        self.elevator_pub.publish(msg)
    
    def pub_flap(self, position):
        msg = Float32()
        msg.data = float(position)
        self.flap_pub.publish(msg)

    ################## helper time compensated move functions ######################
    def pivot_45(self):
        self.pub_path(1, -1, 33/8) #TODO
        self.wait_path()

    def lift_bucket(self):
        self.pub_bucket(100)
        time.sleep(4.5) #TODO
        self.pub_bucket(0)

    def drop_bucket(self):
        self.pub_bucket(-100)
        time.sleep(4.2) #TODO
        self.pub_bucket(0)
    
    def lift_duck(self):
        self.pub_duck(-80) #TODO
        time.sleep(2) #TODO

    def drop_duck(self):
        self.pub_duck(65) #TODO
        time.sleep(1.5) #TODO

    def open_claw(self):
        self.pub_claw(-45) #TODO
        time.sleep(1) #TODO

    def close_claw(self):
        self.pub_claw(-19) #TODO
        time.sleep(1) #TODO

    def lift_elevator(self):
        self.pub_elevator(90) #TODO
        time.sleep(2.0) #TODO

    def drop_elevator(self):
        self.pub_elevator(-80) #TODO
        time.sleep(2.0) #TODO

    def open_flap(self):
        self.pub_flap(-90) #TODO
        time.sleep(1.0) #TODO

    def close_flap(self):
        self.pub_flap(45) #TODO
        time.sleep(1.0) #TODO

    ####################### approach drive functions ###############################
    # drives to x, y relative coordinates
    # percent of path 0.0-1.0
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
            z /= 1.9

            path.left = 1 + z
            path.right = 1 - z
            
            path.distance = abs(r*theta)*percent
        
        self.path_pub.publish(path)
        self.get_logger().info(f"Path {path.left:.3f}, {path.right:.3f}, {path.distance:.3f}")
        self.wait_path()
        self.pub_path(1, 1, 2)
        self.wait_path()

    ####################### main program functions ##########################

    # stacks after object inside chute
    def stack(self):
        self.get_logger().info("Stacking")

        self.open_claw()
        self.drop_elevator()
        
        self.close_claw()
        
        # drive backward incase flap is obstructed
        self.pub_path(-1, -1, 4.5)
        self.wait_path()

        self.open_claw()
        self.close_flap()

        # optional reopen and close claw for better grip
        self.pub_path(1, 1, 1)
        time.sleep(0.2)
        self.close_claw()

        self.lift_elevator()

    def collect_green(self):
        self.get_logger().info("Collecting green")

        self.open_flap()

        # drive forward
        self.pub_path(1, 1, 8) #TODO
        self.wait_path() #TODO

        self.stack()

    def collect_red(self):
        self.get_logger().info("Collecting red")

        # drive to block
        self.pub_path(1, 1, 5) #TODO
        self.wait_path()

        # move flap to hit red
        self.pub_flap(25)
        time.sleep(2)

        self.lift_duck()
        self.drop_duck()


    def collect_sphere(self):
        self.get_logger().info("Collecting sphere")

        self.drop_bucket()
        self.open_flap()

        # knock off ball (drive backward)
        self.pub_path(-1, -1, 7) #TODO
        self.wait_path()
        time.sleep(1)

        self.open_claw()
        self.drop_elevator()

        # drive to ball
        self.pub_path(1, 1, 13) #TODO
        self.wait_path()

        # stack sequence
        self.close_claw()
        
        time.sleep(2)

        # drive backward incase flap is obstructed
        self.pub_path(-1, -1, 4.5)
        self.wait_path()
        time.sleep(0.5)

        self.open_claw()
        self.close_flap()

        # optional reopen and close claw for better grip
        self.pub_path(1, 1, 2)
        time.sleep(0.2)
        self.close_claw()

        self.lift_elevator()

    def collect_all(self):
        self.get_logger().info('entered collect all')
        # collect sphere
        if not self.has_sphere:
            self.has_sphere = True

            self.collect_sphere()

            # swap colors because sphere pulls block
            if self.bottom_color == Color.GREEN:
                self.bottom_color = Color.RED
            else:
                self.bottom_color = Color.GREEN
        else:
            #net movement from sphere collection
            self.pub_path(1, 1, 3.5) #TODO
            self.wait_path()
        # collect in proper order
        if self.bottom_color == Color.GREEN:
            self.collect_green()
            self.pub_path(1, 1, 4)
            self.wait_path()
            self.collect_red()
        else:
            self.close_flap()
            time.sleep(1)
            self.collect_red()
            self.pub_path(1, 1, 4)
            self.wait_path()
            self.collect_green()

    def approach(self):
        self.get_logger().info("Approach started")

        # find position closest block
        self.bottom_color = self.closest_block()
        x, y = self.block_coordinates(self.bottom_color)
        distance = math.sqrt(x**2 + y**2)

        # log move
        self.get_logger().info(f"Closest color is {"green" if self.bottom_color==Color.GREEN else "red"} at {x}, {y}: distance {distance}")

        DIST_THRESH = 30 # inches
        DIST_RECALC = 15 # inches left <30

        if distance > DIST_THRESH:
            self.get_logger().info('second detect entered')
            # drive about 10 inches from block
            percent = (distance-DIST_RECALC)/distance
            self.drive_to(x, y, percent)
            time.sleep(1.5)

            # get new block detect data
            # TODO check if valid block seen
            self.wait_detect()

            iter = 0
            while not self.block_visible():
                if iter>=2:
                    self.get_logger().info('second detect failed')
                    return False
                    #break and search
                iter+=1
                self.wait_detect()

            # recalculate position of closest block
            self.bottom_color = self.closest_block()
            x, y = self.block_coordinates(self.bottom_color)

            # relog for double take
            self.get_logger().info(f"Closest color is {"green" if self.bottom_color==Color.GREEN else "red"} at {x}, {y}: distance {distance}")

        self.get_logger().info('about to drive')

        # drive all the way to the block
        self.drive_to(x, y, 1)

        # delay
        self.pub_path(1, 1, 1)
        return True
        

    def search(self):
        self.get_logger().info("Search started")

        # for now just spinning
        # TODO add move searching if needed (probably not)
        self.pub_path(1,1,-4)
        self.wait_path()
        # spin until sees block
        while True:
            if self.block_visible():
                break

            # spin 45 degrees
            self.pivot_45()

            # get and wait for new data from cv
            self.wait_detect()

    def deploy(self):
        self.get_logger().info("Deploy started")

        # TODO drive towards orange wall


    def run(self):
        self.get_logger().info("Program started")

        self.startTime = time.time()
        self.started = True

        # self.collect_all()
        # self.collect_green()
        # self.collect_red()

        self.pub_path(1, 1, 6)
        time.sleep(1.5)
        # self.pub_path(-1, -1, 4)
        # time.sleep(1)

        # # grab next CV
        self.wait_detect()

        # self.search()
        # self.approach()
        # self.jerk_pub.publish(Empty())

        for i in range(100):
            self.search()
            complete = self.approach()
            if not complete:
                continue
            self.get_logger().info('going to collect')
            self.collect_all()
        
        self.deploy()

        self.get_logger().info("Program ended")

    # program entry point
    def start_callback(self, goal):
        run_thread = threading.Thread(target=self.run, args=(), daemon=True)
        run_thread.start()

        # close ros action server
        goal.succeed()
        return Start.Result()

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