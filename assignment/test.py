import cv2
import pygame
import time
from picamera2 import Picamera2
import numpy as np
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

# Constants
_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 2
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

width, height = 1280, 960  # Reduce resolution for better FPS
lwidth, lheight = 320, 240  # Reduce resolution for better FPS

# FPS parameters
fps_avg_frame_count = 10

def visualize(image: np.ndarray, detection_result: processor.DetectionResult) -> np.ndarray:
    """Draws bounding boxes on the input image."""
    for detection in detection_result.detections:
        category = detection.categories[0]
        category_name = category.category_name
        if category_name == 'stop sign':
            print("stop sign")
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

# Initialize object detection model with optimizations
base_options = core.BaseOptions(file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
detection_options = processor.DetectionOptions(max_results=1, score_threshold=0.5)  # Limit to 1 result for speed
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

# Initialize the camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format":"RGB888", "size": (width, height)}, lores={"size": (lwidth, lheight)},  display="lores")
picam2.align_configuration(config)
picam2.configure(config)
picam2.start()

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((width, height))
screen = pygame.display.set_mode((lwidth, lheight))
pygame.display.set_caption("Object Detection Stream")

# FPS calculation variables
counter, fps = 0, 0
start_time = time.time()

# Main loop
running = True
while running:
    # Capture frame from the camera
    image = picam2.capture_array("lores")
    image = cv2.flip(image, 0)

    # Calculate FPS
    counter += 1
    if counter % fps_avg_frame_count == 0:
        end_time = time.time()
        fps = fps_avg_frame_count / (end_time - start_time)
        start_time = time.time()

    # Convert to RGB for TensorFlow model
    # rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection
    detection_result = detector.detect(input_tensor)
    image = visualize(image, detection_result)

    # Display FPS
    fps_text = f'FPS = {fps:.1f}'
    cv2.putText(image, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, _TEXT_COLOR, 2)

    # Convert image from BGR to RGB format required by Pygame (already in RGB format for TFLite)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # print(fps_text)
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

# Clean up and exit
pygame.quit()
