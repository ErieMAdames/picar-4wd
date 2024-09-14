import numpy as np
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
    speed = 30
    width = 640 * 2
    height = 480 * 2

    def __init__(self):
        print('Starting camera stream')

    def scan(self):
        global frame, pygame_frame
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (self.width, self.height)}))
        picam2.start()

        # FPS calculation variables
        prev_frame_time = time.time()

        while True:
            # Capture frame from camera
            image = picam2.capture_array("main")

            # Flip the image and convert to RGB
            image = cv2.flip(image, 1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Calculate FPS
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time

            # Display FPS on the frame
            fps_text = 'FPS = {:.1f}'.format(fps)
            cv2.putText(image, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

            # Update global frames for Flask and Pygame
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
        print('Starting camera stream with Flask and Pygame')

        # Create a Map object
        map_instance = Map()

        # Create threads for Flask server and Pygame display
        flask_thread = threading.Thread(target=run_flask)
        pygame_thread = threading.Thread(target=run_pygame)
        flask_thread.start()
        pygame_thread.start()

        # Run the scan method in the main thread
        map_instance.scan()

    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pygame.quit()
