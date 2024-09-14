import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
import math
from functools import reduce
import sys

class AvoidObjects():
    current_car_angle = 0
    speed = 30
    turning_time = .9
    current_angle = 0
    us_step = pc4.STEP
    min_angle = -36
    max_angle = 36
    distances = []
    WHEEL_DIAMETER = 0.0662  # Example wheel diameter in meters
    PPR = 20  # Example pulses per revolution
    LEFT_ENCODER_PIN = 25  # Replace with your GPIO pin number
    RIGHT_ENCODER_PIN = 4  # Replace with your GPIO pin number
    left_encoder_count = 0
    right_encoder_count = 0
    turning_angle = 0.0  # Initial angle in degrees
    forward_dist = .75
    read = False
    angle_offset = -10
    # Setup GPIO
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.LEFT_ENCODER_PIN, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_PIN, GPIO.RISING, callback=self.right_encoder_callback)
        print('starting')
        traveled = self.go_distance(10, True)
        if traveled < 1:
            retrace_steps = self.avoid()
            exit()
    # Variables to store encoder counts
    # Callback functions to increment counts
    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel): 
        self.right_encoder_count += 1

    # # Add event detection for rising edges
    def scan(self):
        self.current_angle = 90 if self.current_angle > 0 else -90
        self.us_step = -pc4.STEP if self.current_angle > 0 else pc4.STEP
        for _ in range(5):
            self.current_angle += self.us_step
            if self.current_angle >= self.max_angle:
                self.current_angle = self.max_angle
                self.us_step = -pc4.STEP
            elif self.current_angle <= self.min_angle:
                self.current_angle = self.min_angle
                self.us_step = pc4.STEP
            distance = pc4.get_distance_at(self.current_angle)
            self.distances.append(distance)
        if self.us_step < 0:
            self.distances.reverse()
        distances_map = map(lambda x: x < 20 and x != -2 and x != -1, self.distances)
        stop = reduce(lambda x, y: x or y, distances_map)
        if stop:
            print(self.distances)
        self.distances = []
        return stop

    def avoid(self, right=True):
        # if there is something in the way while avoiding, retrace stepts
        print('avoiding')
        # retrace_steps = []
        self.turn_right()
        # turning_right = 'turning right' if right else 'turning left'
        # turning_left = 'turining left' if right else 'turningright'
        # print(turning_right)
        stop = self.scan()
        # time.sleep(.5)
        # retrace_steps.append((not right, 0))
        if stop:
            print('obstacle')
            # turn right to retrace
            return #retrace_steps
        # print('going')
        dist = self.go_distance(self.forward_dist, True)
        # print(dist)
        # retrace_steps.append(('b', dist))
        if dist < self.forward_dist:
            print('obstacle')
            return #retrace_steps
        self.turn_left()
        # print(turning_left)
        stop = self.scan()
        # retrace_steps.append((right, 0))
        if stop:
            print('obstacle')
            return #retrace_steps
        print('going')
        dist = self.go_distance(self.forward_dist, True)
        # retrace_steps.append(('b', dist))
        if dist < self.forward_dist:
            # print('obstacle')
            return #retrace_steps
        self.turn_left()
        # print(turning_left)
        stop = self.scan()
        # retrace_steps.append((right, 0))
        if stop:
            print('obstacle')
            return #retrace_steps
        # print('going')
        dist = self.go_distance(self.forward_dist, True)
        # retrace_steps.append(('b', dist))
        if dist < self.forward_dist:
            print('obstacle')
            return #retrace_steps
        self.turn_right()
        # print(turning_right)
        stop = self.scan()
        # retrace_steps.append((not right, 0))
        if stop:
            print('obstacle')
            return #retrace_steps
        # print('going')
        dist = self.go_distance(self.forward_dist, True)
        # retrace_steps.append(('b', dist))
        if dist < self.forward_dist:
            print('obstacle')
            return #retrace_steps
        # return ##[]
    def retrace(self, retrace_steps):
        print('retracing')
        for step in reversed(retrace_steps):
            if step[0] == 'b':
                self.go_distance(step[1], False)
            else:
                self.turn(step[0], 90, self.speed)
                

    def go_distance(self, dist, forward=True):
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        def calculate_distance(counts):
            wheel_circumference = self.WHEEL_DIAMETER * 3.14159
            distance = (counts / self.PPR) * wheel_circumference
            return distance

        left_distance = calculate_distance(self.left_encoder_count)
        right_distance = calculate_distance(self.right_encoder_count)
        
        while left_distance < dist or right_distance < dist:
            self.calculate_angles()
            stop = self.scan()
            if stop:
                break
            if forward:
                pc4.forward(self.speed)
            else:
                pc4.backward(self.speed)
            left_distance = calculate_distance(self.left_encoder_count)
            right_distance = calculate_distance(self.right_encoder_count)
            # print(left_distance)
            # print(right_distance)
            # print('----')
        pc4.stop()
        time.sleep(.5)
        return min(left_distance, right_distance)

    def turn_right(self,  angle=90, speed=30):
        pc4.turn_right(speed)
        input()
        pc4.stop()
    def turn_left(self, angle=-90, speed=30):
        pc4.turn_left(speed)
        input()
        pc4.stop()

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')
        AvoidObjects()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()  # Clean up GPIO on exit