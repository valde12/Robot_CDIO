from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.C)
small_motor = Motor(Port.B)

# Create a DriveBase object with the initialized motors
# Adjust the wheel diameter and axle track according to your robot design
robot = DriveBase(left_motor, right_motor, wheel_diameter=40, axle_track=110)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)

small_motor.run(-300)
wait(10000)
small_motor.run(700)
wait(10000)
# Stop the motors at the end
robot.stop()