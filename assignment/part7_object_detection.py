import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
import math
from functools import reduce
import sys
import cv2
import pygame
import time
from picamera2 import Picamera2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

class DetectObject():
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
    base_options = core.BaseOptions(file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
    detection_options = processor.DetectionOptions(max_results=1, score_threshold=0.5)  # Limit to 1 result for speed
    options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)
    picam2 = Picamera2()
    width, height = 320, 240  # Reduce resolution for better FPS
    config = picam2.create_preview_configuration(main={"format":"RGB888", "size": (width, height)})
    picam2.align_configuration(config)
    picam2.configure(config)
    picam2.start()
    # Setup GPIO
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.LEFT_ENCODER_PIN, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_PIN, GPIO.RISING, callback=self.right_encoder_callback)
        print('starting')
        traveled = self.go_distance(10, True)
        
    # Variables to store encoder counts
    # Callback functions to increment counts
    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel): 
        self.right_encoder_count += 1

                
    def detect(self):
        image = self.picam2.capture_array("main")
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        return self.detector.detect(input_tensor).detections
    def detect_person(self):
        image = self.picam2.capture_array("main")
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        for detection in  self.detector.detect(input_tensor).detections:
            if detection.categories[0].index == 0:
                return True
    def go_distance(self, dist, forward=True):
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        def calculate_distance(counts):
            wheel_circumference = self.WHEEL_DIAMETER * 3.14159
            distance = (counts / self.PPR) * wheel_circumference
            return distance

        left_distance = calculate_distance(self.left_encoder_count)
        right_distance = calculate_distance(self.right_encoder_count)
        stopped = False
        while left_distance < dist or right_distance < dist:
            for detection in self.detect():
                if not stopped:
                    if detection.categories[0].index == 12 and (detection.bounding_box.width >= 100 or detection.bounding_box.height >=100):
                        stopped = True
                        pc4.stop()
                        time.sleep(3)
                if detection.categories[0].index == 0:
                    pc4.stop()
                    while self.detect_person():
                        time.sleep(1)
            pc4.forward(self.speed)
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
        DetectObject()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()  # Clean up GPIO on exit