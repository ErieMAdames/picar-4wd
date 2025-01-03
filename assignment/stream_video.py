import numpy as np
import time
import sys
import cv2
import argparse
import threading
from flask import Flask, Response, request
import pygame
import os
from picamera2 import Picamera2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
# Initialize the Flask app
app = Flask(__name__)

# Set up global variables for streaming
frame = None
pygame_frame = None

# Event to signal the threads to stop
stop_event = threading.Event()

np.set_printoptions(threshold=sys.maxsize)
square_size_ = 20
width, height = 1280 , 960
class Map:
    # base_options = core.BaseOptions(file_name='efficientdet_lite0.tflite', use_coral=True, num_threads=4)
    # detection_options = processor.DetectionOptions(max_results=4, score_threshold=0.5)  # Limit to 1 result for speed
    # options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
    # detector = vision.ObjectDetector.create_from_options(options)
    def __init__(self):
        print('Starting camera stream')

    def scan(self):
        global frame, pygame_frame
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format":"RGB888", "size": (width, height)})
        picam2.align_configuration(config)
        picam2.configure(config)
        picam2.start()

        # FPS calculation variables
        prev_frame_time = time.time()

        while not stop_event.is_set():
            # Capture frame from camera
            image = picam2.capture_array("main")

            # Flip the image and convert to RGB
            image = cv2.flip(image, 0)
            # rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # input_tensor = vision.TensorImage.create_from_array(rgb_image)
            # detection_result =  self.detector.detect(input_tensor)
            # image = self.visualize(image, detection_result)
            # Calculate FPS
            center_x, center_y = width // 2, height // 2
            square_size = min(width, height) // square_size_  # Adjust the size of the square if needed
            top_left_x = center_x - square_size // 2
            top_left_y = center_y - square_size // 2
            bottom_right_x = center_x + square_size // 2
            bottom_right_y = center_y + square_size // 2

            # Calculate center contrast
            contrast_value = self.calculate_center_contrast(image)

            # Draw a red square around the center square
            cv2.rectangle(image, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 0, 255), 2)
            center_square = image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]

            # Resize the center square to 4x its original size
            enlarged_center_square = cv2.resize(center_square, (square_size * 4, square_size * 4), interpolation=cv2.INTER_LINEAR)

            # Define the top right corner position for overlay
            top_right_y = 10  # 10 pixels padding from the top
            top_right_x = width - (square_size * 4) - 10  # 10 pixels padding from the right

            # Overlay the enlarged center square onto the top right corner of the frame
            image[top_right_y:top_right_y + square_size * 4, top_right_x:top_right_x + square_size * 4] = enlarged_center_square
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time

            # # Display FPS on the frame
            fps_text = 'FPS = {:.1f}'.format(fps) + f"Center Contrast Value: {contrast_value}"
            cv2.putText(image, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
            image = cv2.flip(image, 1)
            # Update global frames for Flask and Pygame
            frame = cv2.flip(image, 1)
            pygame_frame = image

        picam2.stop()
    def calculate_center_contrast(self, image: np.ndarray) -> float:
        blur = 9
        """Calculates the contrast of a central square region in the image using Laplacian variance."""
        center_x, center_y = width // 2, height // 2
        square_size = min(width, height) // square_size_  # Adjust size of the center square as needed

        # Define the bounding box for the center square
        top_left_x = center_x - square_size // 2
        top_left_y = center_y - square_size // 2
        bottom_right_x = center_x + square_size // 2
        bottom_right_y = center_y + square_size // 2

        # Extract the center square region
        center_region = image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]
        center_region = cv2.medianBlur(center_region, blur)

        # Convert to grayscale
        gray_center = cv2.cvtColor(center_region, cv2.COLOR_RGB2GRAY)

        # Calculate the Laplacian variance
        laplacian_var = cv2.Laplacian(gray_center, cv2.CV_64F).var()
        return laplacian_var

    def visualize(self, image: np.ndarray, detection_result: processor.DetectionResult) -> np.ndarray:
        """Draws bounding boxes on the input image."""
        # Constants
        _MARGIN = 10  # pixels
        _ROW_SIZE = 10  # pixels
        _FONT_SIZE = 2
        _FONT_THICKNESS = 1
        _TEXT_COLOR = (0, 0, 255)  # red
        for detection in detection_result.detections:
            # if detection.categories[0].index == 12:
            #     print("STOP!!!")
            category = detection.categories[0]
            # print(category)
            category_name = category.category_name
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)
            # Draw label and score
            probability = round(category.score, 2)
            result_text = f"{category_name} ({probability})"
            text_location = (_MARGIN + bbox.origin_x, _MARGIN + _ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
        return image

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
    screen = pygame.display.set_mode((width, height))  # Same resolution as the camera stream
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
