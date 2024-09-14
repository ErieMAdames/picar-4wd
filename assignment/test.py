import cv2
import pygame
import time
import threading
import numpy as np
from picamera2 import Picamera2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

# Constants
_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 2
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

running = True
width, height = 640, 480  # Reduce resolution for better FPS

# FPS parameters
fps_avg_frame_count = 10

# Initialize object detection model with optimizations
base_options = core.BaseOptions(file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
detection_options = processor.DetectionOptions(max_results=1, score_threshold=0.5)  # Limit to 1 result for speed
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (width, height)}))
picam2.start()

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Object Detection Stream")

# Shared variables
frame = None
frame_lock = threading.Lock()

# FPS calculation variables
fps_counter = 0
fps = 0
fps_start_time = time.time()

def capture_frames():
    global frame
    while running:
        with frame_lock:
            frame = picam2.capture_array("main")
            frame = cv2.flip(frame, 0)

def process_frames():
    global frame, fps, fps_start_time, fps_counter
    while running:
        if frame is not None:
            with frame_lock:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                input_tensor = vision.TensorImage.create_from_array(rgb_image)
                
                # Run object detection
                detection_result = detector.detect(input_tensor)
                frame = visualize(frame, detection_result)
                
                # Calculate FPS
                fps_counter += 1
                if fps_counter % fps_avg_frame_count == 0:
                    end_time = time.time()
                    fps = fps_avg_frame_count / (end_time - fps_start_time)
                    fps_start_time = time.time()
                    
                # Display FPS
                fps_text = f'FPS = {fps:.1f}'
                cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, _TEXT_COLOR, 2)

def display_frames():
    global frame
    global running
    while running:
        if frame is not None:
            with frame_lock:
                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                image = cv2.flip(image, 1)
                
                # Ensure image is of shape (height, width, 3) for Pygame
                if image.shape[2] == 3:
                    # Convert image to 3D surface (pygame expects (width, height, channels))
                    frame_surface = pygame.surfarray.make_surface(np.rot90(image))
                    screen.blit(frame_surface, (0, 0))
                    pygame.display.update()

        # Check for quit events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

def visualize(image: np.ndarray, detection_result: processor.DetectionResult) -> np.ndarray:
    """Draws bounding boxes on the input image."""
    for detection in detection_result.detections:
        category = detection.categories[0]
        category_name = category.category_name
        if category_name == 'stop sign':
            bbox = detection.bounding_box
            start_point = bbox.origin_x, bbox.origin_y
            end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
            cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)
            # Draw label and score
            probability = round(category.score, 2)
            result_text = f"{category_name} ({probability})"
            text_location = (_MARGIN + bbox.origin_x, _MARGIN + _ROW_SIZE + bbox.origin_y)
            cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
    return image

# Main loop setup
capture_thread = threading.Thread(target=capture_frames)
process_thread = threading.Thread(target=process_frames)
display_thread = threading.Thread(target=display_frames)

# Start threads
capture_thread.start()
process_thread.start()
display_thread.start()

# Join threads
capture_thread.join()
process_thread.join()
display_thread.join()

# Clean up and exit
pygame.quit()
