from mindstorms import MSHub, Motor, MotorPair, ColorSensor, DistanceSensor, App
from mindstorms.control import wait_for_seconds, wait_until, Timer
from mindstorms.operator import greater_than, greater_than_or_equal_to, less_than, less_than_or_equal_to, equal_to, not_equal_to
import math

from collections import deque

hub = MSHub()
hub.light_matrix.set_orientation("left")

poll_interval = .1

hub.status_light.on("yellow")

calibration_history = []
while(len(calibration_history) < 25):
    current_roll_angle = hub.motion_sensor.get_roll_angle()

    if(calibration_history.count(current_roll_angle) != len(calibration_history)):
        calibration_history.clear()

    calibration_history.append(current_roll_angle)

    wait_for_seconds(poll_interval)


idle_roll_angle = current_roll_angle
moving_history = [False for x in range(10)]

required_difference = 3

number_of_hits = 0
hub.light_matrix.write(number_of_hits)

while(True):
    current_roll_angle = hub.motion_sensor.get_roll_angle()
    
    if(abs(current_roll_angle - idle_roll_angle) >= required_difference):
        moving = True
    else:
        moving = False

    old_idle = moving_history.count(True) == 0

    moving_history.append(moving)
    moving_history.pop(0)

    new_idle = moving_history.count(True) == 0

    hub.status_light.on("green" if new_idle else "red")

    if(old_idle and not new_idle):
        hub.speaker.beep(volume=100)

        number_of_hits += 1
        hub.light_matrix.write(number_of_hits)

    wait_for_seconds(poll_interval)
