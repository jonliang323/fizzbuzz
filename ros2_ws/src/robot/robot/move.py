#!/usr/bin/env python3
# Run this in one terminal
# colcon build
# ros2 run robot move
# Then use these in another terminal
# ros2 topic pub /drive interfaces/msg/Drive "{command: 'forward', value: 20}" --once
# ros2 topic pub /lift std_msgs/msg/Float32 "{data: 0}" --once
# ros2 topic pub /stick std_msgs/msg/String "{data: 'down'}" --once
# ros2 topic pub /path interfaces/msg/Path "{left: 1, right: 1, distance: 1000}" --once
# ros2 topic pub /path interfaces/msg/Path "{left: 1, right: -1, distance: 500}" --once
# ros2 topic pub /path interfaces/msg/Path "{left: 1, right: 1, distance: 1000}" --once
# ros2 topic pub /path interfaces/msg/Path "{left: 1, right: -1, distance: 500}" --once
# ros2 launch robot camera_launch.py
# ros2 topic pub /path interfaces/msg/Path"{left: 1.0, right: 1.0, distance: 5000}" --once
import math
import time

import numpy as np
import rclpy
import RPi.GPIO as GPIO
from gpiozero import Button
from interfaces.msg import Path
from raven import Raven
from rclpy.node import Node
from std_msgs.msg import Float32, String

TICK_ROT = 640

WHEEL_D = 78.0
BASE_D = 241.0
BASE_RATIO = WHEEL_D / BASE_D

MAX_VEL = 2 * TICK_ROT
ACCEL = 1 * TICK_ROT

TURN_CONST = BASE_RATIO * 2 * math.pi / TICK_ROT
ANGLE_PROP = 800

LEFT_MOTOR = Raven.MotorChannel.CH4
RIGHT_MOTOR = Raven.MotorChannel.CH1
BUCKET_MOTOR = Raven.MotorChannel.CH3

class MoveNode(Node):
    def __init__(self):
        super().__init__('move')

        self.angle = 0.0
        # Variables initialized at beginning of movement
        self.start_angle = 0.0
        self.start_left = 0.0
        self.start_right = 0.0
        self.total_dist = 0.0

        self.left_coef = 0
        self.right_coef = 0

        # Dynamic
        self.last_speed = 0.0
        self.current_dist = 0.0

        self.angle_updated = False
        self.raven = Raven()
        # setup motor
        for motor in [LEFT_MOTOR, RIGHT_MOTOR]:
            self.raven.set_motor_encoder(motor, 0)
            self.raven.set_motor_mode(motor, Raven.MotorMode.POSITION)
            self.raven.set_motor_torque_factor(motor, 80)
            self.raven.set_motor_pid(motor, p_gain=20, i_gain=3, d_gain=.1)
            self.raven.set_motor_target(motor, 0)

        # dump truck pid position
        self.raven.set_motor_encoder(BUCKET_MOTOR, 0)
        self.raven.set_motor_mode(BUCKET_MOTOR, Raven.MotorMode.POSITION)
        self.raven.set_motor_torque_factor(BUCKET_MOTOR, 80)
        self.raven.set_motor_pid(BUCKET_MOTOR, p_gain=20, i_gain=3, d_gain=.1)
        self.raven.set_motor_target(BUCKET_MOTOR, 0)

        self.create_subscription(Float32, 'imu', self.imu_callback, 10)
        self.create_subscription(Float32, 'bucket', self.bucket_callback, 10)
        self.create_subscription(Path, 'path', self.path_callback, 10)

        self.create_subscription(Float32, 'duck', self.duck_callback, 10)
        self.create_subscription(Float32, 'claw', self.claw_callback, 10)
        self.create_subscription(Float32, 'elevator', self.elevator_callback, 10)
        self.create_subscription(Float32, 'flap', self.flap_callback, 10)

        self.path_timer = self.create_timer(.01, self.update_path)

        self.get_logger().info("Move node has started")


    def duck_callback(self, msg):
        self.raven_board.set_servo_position(Raven.ServoChannel.CH1, msg.data, 500, 2500)

    def claw_callback(self, msg):
        self.raven_board.set_servo_position(Raven.ServoChannel.CH2, msg.data, 500, 2500)
    
    def elevator_callback(self, msg):
        self.raven_board.set_servo_position(Raven.ServoChannel.CH3, msg.data, 500, 2500)
    
    def flap_callback(self, msg):
        self.raven_board.set_servo_position(Raven.ServoChannel.CH4, msg.data, 500, 2500)

    def imu_callback(self, msg):
        self.angle = msg.data
        if not self.angle_updated:
            self.start_angle = self.angle
            self.angle_updated = True

    def bucket_callback(self, msg):
        self.raven.set_motor_target(BUCKET_MOTOR, msg.data)

    def path_callback(self, msg):
        self.start_path(-msg.left, msg.right, msg.rotations)

    def start_path(self, left_coef, right_coef, distance):
        self.total_dist = distance
        self.left_coef = left_coef
        self.right_coef = right_coef
        self.start_angle = self.angle
        self.current_dist = 0
        self.start_left = self.raven.get_motor_encoder(LEFT_MOTOR)
        self.start_right = self.raven.get_motor_encoder(RIGHT_MOTOR)

    def update_path(self):
        if not self.angle_updated:
            return

        DT = .01
        delta_speed = ACCEL * DT

        target_speed = 0
        # Do i need to slow down? If yes, starts slowing down, else speed up or stay same
        if((self.total_dist - self.current_dist) <= (self.last_speed ** 2) / (2 * ACCEL)):
           target_speed = max(self.last_speed - delta_speed, 0)
        else:
            target_speed = min(self.last_speed + delta_speed, MAX_VEL)

        self.current_dist += (self.last_speed + target_speed) / 2 * DT
        self.last_speed = target_speed

        target_angle = (self.right_coef + self.left_coef)/2.0 * TURN_CONST * self.current_dist

        angle_error = (self.angle - self.start_angle) - target_angle

        self.start_left -= angle_error * ANGLE_PROP * DT
        self.start_right -= angle_error * ANGLE_PROP * DT

        self.raven.set_motor_target(LEFT_MOTOR, self.start_left + self.current_dist * self.left_coef)
        self.raven.set_motor_target(RIGHT_MOTOR, self.start_right + self.current_dist * self.right_coef)

def destroy_node(self):
    self.raven.set_motor_mode(LEFT_MOTOR, Raven.MotorMode.DISABLE)
    self.raven.set_motor_mode(RIGHT_MOTOR, Raven.MotorMode.DISABLE)
    self.raven.set_motor_mode(BUCKET_MOTOR, Raven.MotorMode.DISABLE)

    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down move node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()