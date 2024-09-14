import numpy as np
import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
import sys
import cv2
from flask import Flask, Response
import threading
from picamera2 import Picamera2
import pygame

# Initialize the Flask app
app = Flask(__name__)

# Set up global variables for streaming
frame = None
pygame_frame = None

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
    angle_offset = -10
    width = 640 * 2
    height = 480 * 2

    # Setup GPIO
    def __init__(self):
        print('starting')

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
        global frame, pygame_frame
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (self.width, self.height)}))
        picam2.start()
        counter, fps = 0, 0
        fps_avg_frame_count = 10
        # Visualization parameters
        row_size = 20  # pixels
        left_margin = 24  # pixels
        text_color = (0, 0, 255)  # red
        font_size = 1
        font_thickness = 1
        fps_avg_frame_count = 10
        start_time = time.time()
        while True:
            image = picam2.capture_array("main")
            if counter % fps_avg_frame_count == 0:
                end_time = time.time()
                fps = fps_avg_frame_count / (end_time - start_time)
                start_time = time.time()
            image = cv2.flip(image, 1)
            image = cv2.flip(image, 0)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            fps_text = 'FPS = {:.1f}'.format(fps)
            text_location = (left_margin, row_size)

            cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        font_size, text_color, font_thickness)
            # Prepare the frame for streaming and pygame display
            frame = image
            pygame_frame = image

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

def run_pygame():
    global pygame_frame
    pygame.init()
    screen = pygame.display.set_mode((1280, 960))  # Same resolution as the camera stream
    pygame.display.set_caption("Camera Stream")
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        if pygame_frame is not None:
            frame_rgb = cv2.cvtColor(pygame_frame, cv2.COLOR_BGR2RGB)
            frame_rgb = np.rot90(frame_rgb)  # Rotate if needed for correct orientation
            frame_surface = pygame.surfarray.make_surface(frame_rgb)
            screen.blit(frame_surface, (0, 0))

        pygame.display.update()
        clock.tick(30)  # 30 FPS cap for Pygame

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')

        # Create a Map object
        map_instance = Map()

        # Create threads for Flask server and Pygame display
        flask_thread = threading.Thread(target=run_flask)
        pygame_thread = threading.Thread(target=run_pygame)
        flask_thread.start()
        pygame_thread.start()

        # Run the scan method in the main thread
        while True:
            map_instance.scan()
            input()

    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()
        pygame.quit()
