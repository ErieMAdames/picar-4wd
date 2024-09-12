import numpy as np
import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
import sys
import pygame
import cv2
from flask import Flask, Response
import threading

# Initialize the Flask app
app = Flask(__name__)

# Set up global variables for streaming
frame = None

np.set_printoptions(threshold=sys.maxsize)

class Map:
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
        # pygame.init()
        # self.screen = pygame.display.set_mode((500, 500))

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

    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel): 
        self.right_encoder_count += 1

    def scan(self):
        global frame
        print('scanning')
        self.current_angle = -90
        self.us_step = 1
        grid_size = 100
        map_grid = np.zeros((grid_size, grid_size), dtype=int)
        for _ in range(180):
            distance = pc4.get_distance_at(self.current_angle)
            if distance > 0:
                dx = int(distance * np.cos(np.radians(self.current_angle + 90))) + 50
                dy = int(distance * np.sin(np.radians(self.current_angle + 90)))# + 50
                
                if 0 <= dx < 100 and 0 <= dy < 100:
                    map_grid[dy, dx] = 1

                image = np.zeros((100, 100, 3), dtype=np.uint8)
                image[map_grid == 0] = [0, 255, 0]
                image[map_grid == 1] = [255, 0, 0]
                enlarged_image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_NEAREST)
                rotated_image = cv2.rotate(enlarged_image, cv2.ROTATE_90_CLOCKWISE)
                frame_surface = pygame.surfarray.make_surface(rotated_image)

                # self.screen.blit(frame_surface, (0, 0))
                # pygame.display.update()
                frame = cv2.flip(cv2.rotate(cv2.cvtColor(pygame.surfarray.array3d(frame_surface), cv2.COLOR_RGB2BGR), cv2.ROTATE_90_CLOCKWISE), 1)  # Correct the rotation for streaming

            self.current_angle += self.us_step
            self.distances.append(distance)

        time.sleep(10)
        for x in np.flip(map_grid, 0):
            x_str = np.array_repr(x).replace('\n', '').replace(' ', '').replace('array([', '').replace('])', '').replace('0','_').replace('1','@')
            print(x_str)
        self.distances = []

    def turn_right(self, angle=90, speed=30):
        pc4.turn_right(speed)
        time.sleep(self.turning_time)
        pc4.stop()

    def turn_left(self, angle=90, speed=30):
        pc4.turn_left(speed)
        time.sleep(self.turning_time)
        pc4.stop()

def generate_frames():
    global frame
    while True:
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.1)  # Control the frame rate

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')

        # Create a Map object
        map_instance = Map()

        # Create a thread for the Flask server
        flask_thread = threading.Thread(target=run_flask)
        flask_thread.start()

        # Run the scan method in the main thread
        map_instance.scan()

    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()
