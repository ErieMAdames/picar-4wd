import numpy as np
import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
from functools import reduce
import sys
import pygame
import cv2
np.set_printoptions(threshold=sys.maxsize)
class Map():
    current_car_angle = 0
    speed = 30
    turning_time = .9
    current_angle = 0
    us_step = 1
    min_angle = -90
    max_angle = 90
    distances = []
    WHEEL_DIAMETER = 0.0662  # Example wheel diameter in meters
    PPR = 20  # Example pulses per revolution
    LEFT_ENCODER_PIN = 25  # Replace with your GPIO pin number
    RIGHT_ENCODER_PIN = 4  # Replace with your GPIO pin number
    left_encoder_count = 0
    right_encoder_count = 0
    forward_dist = .5
    # Setup GPIO
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.LEFT_ENCODER_PIN, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_PIN, GPIO.RISING, callback=self.right_encoder_callback)
        print('starting')
        pygame.init()
        self.screen = pygame.display.set_mode((500, 500))
        self.scan()
    def calibrate_turn_speed(self):
        start = time.time()
        try:
            while True:
                pc4.turn_left(self.speed)
        except KeyboardInterrupt:
            pc4.stop()
            self.turning_time = time.time() - start
            pc4.turn_right(self.speed)
            time.sleep(self.turning_time)
            pc4.stop()
    # Variables to store encoder counts
    # Callback functions to increment counts
    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel): 
        self.right_encoder_count += 1

    # # Add event detection for rising edges
    def scan(self):
        print('scanning')
        self.current_angle = -90
        self.us_step = 1
        grid_size = 100
        map_grid = np.zeros((grid_size, grid_size), dtype=int)
        for _ in range(180):
            print(self.current_angle)
            distance = pc4.get_distance_at(self.current_angle)
            if distance > 0:
                dx = int(distance * np.cos(np.radians(self.current_angle + 90))) + 50
                dy = int(distance * np.sin(np.radians(self.current_angle + 90))) + 50
                
                # Check if the calculated position is within the grid
                if dx >= 0 and dy >= 0 and dx < 100 and dy < 100:
                    map_grid[dy, dx] = 1  # Mark the cell as an obstacle
                image = np.zeros((100, 100, 3), dtype=np.uint8)
                image[map_grid == 0] = [255, 0, 0]  # Blue for 0
                image[map_grid == 1] = [0, 0, 255]  # Red for 1
                enlarged_image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_NEAREST)

                # Rotate the image 90 degrees clockwise
                rotated_image = cv2.rotate(enlarged_image, cv2.ROTATE_90_CLOCKWISE)
                frame_surface = pygame.surfarray.make_surface(rotated_image)
                # frame_surface = pygame.transform.rotate(frame_surface, -90)
                # frame_surface = pygame.transform.flip(frame_surface, True, False)

                # Display the frame on the pygame window
                self.screen.blit(frame_surface, (0, 0))
                pygame.display.update()
            self.current_angle += self.us_step
            self.distances.append(distance)

        time.sleep(10)
        for x in np.flip(map_grid, 0):
            x_str = np.array_repr(x).replace('\n', '').replace(' ', '').replace('array([', '').replace('])', '').replace('0','_').replace('1','@')
            # print(x)
            print(x_str)
        # if self.us_step < 0:
        #     self.distances.reverse()
        # distances_map = map(lambda x: x < 20 and x >= 0, self.distances)
        # stop = reduce(lambda x, y: x or y, distances_map)
        # if stop:
        #     print(self.distances)
        self.distances = []
    def turn_right(self,  angle=90, speed=30):
        pc4.turn_right(speed)
        time.sleep(self.turning_time)
        pc4.stop()

    def turn_left(self, angle=90, speed=30):
        pc4.turn_left(speed)
        time.sleep(self.turning_time)
        pc4.stop()

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')
        Map()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()  # Clean up GPIO on exit