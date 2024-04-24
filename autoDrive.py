#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import cv2
import numpy as np

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Initialize the motors connected to the wheels
left_motor = Motor(Port.A)
right_motor = Motor(Port.C)

# Create a DriveBase object with the initialized motors
# Adjust the wheel diameter and axle track according to your robot design
robot = DriveBase(left_motor, right_motor, wheel_diameter=40, axle_track=110)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)

gyro_sensor = GyroSensor(Port.S1)

def get_ball_position():
    # OpenCV Kode:
    # F.eks:
    # Capture a frame and process it to get the ball position
    # ball_x, ball_y = process_frame(capture_frame())
    ball_x, ball_y = 0, 0  # Erstat denne linje med kode for at finde bolden
    return ball_x, ball_y


def drive_towards_ball(target_x, current_x, robot_speed, angle_correction):
    if target_x is not None:
        # Calculate the difference between the target and current positions
        error = target_x - current_x

        # Adjust the steering based on the error (proportional control)
        correction = angle_correction * error

        # Drive the robot with the adjusted steering
        robot.drive(robot_speed, correction)
        wait(10)

# angle = degrees to turn, speed = mm/s
def turn(angle, speed):
    gyro_sensor.reset_angle(0)
    if angle < 0:
        while gyro_sensor.angle() > angle:
            left_motor.run(speed=speed)
            right_motor.run(speed=(-1 * speed))
            wait(10)
    elif angle > 0:
        while gyro_sensor.angle() < angle:
            left_motor.run(speed=(-1 * speed))
            right_motor.run(speed=speed)
            wait(10)
    else:
        print("Error: no angle chosen")

    left_motor.brake()
    right_motor.brake()

# distance = mm, robotSpeed = mm/s
def drive(distance, robot_speed):
    robot.reset()
    gyro_sensor.reset_angle(0)

    PROPORTIONAL_GAIN = 1.1
    if distance < 0:  # move backwards
        while robot.distance() > distance:
            reverse_speed = -1 * robot_speed
            angle_correction = 1 * (PROPORTIONAL_GAIN * gyro_sensor.angle())
            robot.drive(reverse_speed, angle_correction)
            wait(10)
    elif distance > 0:  # move forwards
        while robot.distance() < distance:
            angle_correction = 1 * PROPORTIONAL_GAIN * gyro_sensor.angle()
            robot.drive(robot_speed, angle_correction)
            wait(10)
    robot.stop()

# Main program
robot_speed = 500
drive_distance = 1000

# Drive towards the ball for the specified distance
while robot.distance() < drive_distance:
    # Get the current ball position
    ball_x, ball_y = get_ball_position()

    # Drive towards the ball
    drive_towards_ball(ball_x, ev3.screen.width // 2, robot_speed, PROPORTIONAL_GAIN)

# Stop the robot
robot.stop()

# Turn 180 degrees
turn(180, 200)

# Drive towards the ball again for the specified distance
while robot.distance() < drive_distance:
    # Get the current ball position
    ball_x, ball_y = get_ball_position()

    # Drive towards the ball
    drive_towards_ball(ball_x, ev3.screen.width // 2, robot_speed, PROPORTIONAL_GAIN)

# Stop the robot
robot.stop()