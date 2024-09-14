import numpy as np
import time
import sys
import cv2
import argparse
import threading
from flask import Flask, Response, request
from picamera2 import Picamera2
import pygame
import signal
import os

# Initialize the Flask app
app = Flask(__name__)

# Set up global variables for streaming
frame = None
pygame_frame = None

# Event to signal the threads to stop
stop_event = threading.Event()

np.set_printoptions(threshold=sys.maxsize)

class Map:
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

        while not stop_event.is_set():
            # Capture frame from camera
            image = picam2.capture_array("main")

            # Flip the image and convert to RGB
            image = cv2.flip(image, 0)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Calculate FPS
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time

            # Display FPS on the frame
            fps_text = 'FPS = {:.1f}'.format(fps)
            cv2.putText(image, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
            image = cv2.flip(image, 1)

            # Update global frames for Flask and Pygame
            frame = image
            pygame_frame = image

        picam2.stop()

def generate_frames():
    global frame
    while not stop_event.is_set():
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

@app.route('/shutdown', methods=['POST'])
def shutdown():
    shutdown_func = request.environ.get('werkzeug.server.shutdown')
    if shutdown_func:
        shutdown_func()
    return 'Shutting down Flask server...'

def run_flask():
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, threaded=True)

def run_pygame():
    global pygame_frame
    pygame.init()
    screen = pygame.display.set_mode((1280, 960))  # Same resolution as the camera stream
    pygame.display.set_caption("Camera Stream")
    clock = pygame.time.Clock()

    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_event.set()
                pygame.quit()
                return

        if pygame_frame is not None:
            frame_rgb = cv2.cvtColor(pygame_frame, cv2.COLOR_BGR2RGB)
            frame_rgb = np.rot90(frame_rgb)  # Rotate if needed for correct orientation
            frame_surface = pygame.surfarray.make_surface(frame_rgb)
            screen.blit(frame_surface, (0, 0))

        pygame.display.update()
        clock.tick(30)  # 30 FPS cap for Pygame

    pygame.quit()

def main(stream_flask, display_pygame):
    try:
        print('Starting camera stream with Flask and Pygame')

        # Create a Map object
        map_instance = Map()

        # Create threads for Flask server and Pygame display
        threads = []
        if stream_flask:
            flask_thread = threading.Thread(target=run_flask)
            threads.append(flask_thread)
        if display_pygame:
            pygame_thread = threading.Thread(target=run_pygame)
            threads.append(pygame_thread)

        for thread in threads:
            thread.start()

        # Run the scan method in the main thread
        map_instance.scan()

    except KeyboardInterrupt:
        print('\nStopping...')
        stop_event.set()  # Signal all threads to stop
        if stream_flask:
            shutdown_flask()

    finally:
        for thread in threads:
            thread.join()  # Wait for threads to finish
        print("Program exited gracefully.")

def shutdown_flask():
    """Send a request to Flask to shut it down."""
    import requests
    try:
        requests.post('http://127.0.0.1:5000/shutdown')
    except Exception as e:
        print(f"Error shutting down Flask: {e}")

if __name__ == "__main__":
    # Command-line argument parsing
    parser = argparse.ArgumentParser(description="Camera Stream with Flask and Pygame.")
    parser.add_argument('--flask', action='store_true', help="Stream the video feed via Flask")
    parser.add_argument('--pygame', action='store_true', help="Display the video feed on a Pygame window")
    args = parser.parse_args()

    # Run the main function with the selected options
    main(stream_flask=args.flask, display_pygame=args.pygame)
