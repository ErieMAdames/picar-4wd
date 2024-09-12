import cv2
import time
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import threading

# Visualization parameters
_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 2
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red
width = 640 * 2
height = 480 * 2

# Initialize the Flask app
app = Flask(__name__)
frame = None

def visualize(image: np.ndarray, detection_result: processor.DetectionResult) -> np.ndarray:
    """Draws bounding boxes on the input image and returns it."""
    for detection in detection_result.detections:
        category = detection.categories[0]
        category_name = category.category_name
        bbox = detection.bounding_box
        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
        cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)
        probability = round(category.score, 2)
        result_text = f'{category_name} ({probability})'
        text_location = (_MARGIN + bbox.origin_x, _MARGIN + _ROW_SIZE + bbox.origin_y)
        cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                    _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
    return image

def run_detection():
    global frame
    # Initialize the object detection model
    base_options = core.BaseOptions(
        file_name='efficientdet_lite0.tflite', use_coral=False, num_threads=4)
    detection_options = processor.DetectionOptions(
        max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    # Start the camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (width, height)}))
    picam2.start()

    time.sleep(2)  # Allow the camera to warm up
    counter, fps = 0, 0
    start_time = time.time()

    while True:
        image = picam2.capture_array("main")
        counter += 1
        image = cv2.flip(image, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        input_tensor = vision.TensorImage.create_from_array(rgb_image)
        detection_result = detector.detect(input_tensor)

        # Draw bounding boxes and other visualizations on the image
        image = visualize(image, detection_result)

        # Calculate FPS
        if counter % 10 == 0:
            end_time = time.time()
            fps = 10 / (end_time - start_time)
            start_time = time.time()

        # Show the FPS
        fps_text = f'FPS = {fps:.1f}'
        cv2.putText(image, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, _TEXT_COLOR, 1)

        # Convert image to JPEG format
        ret, jpeg = cv2.imencode('.jpg', image)
        if ret:
            frame = jpeg.tobytes()

        time.sleep(0.1)  # Control the frame rate

def generate_frames():
    global frame
    while True:
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == "__main__":
    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    # Run the object detection and streaming function
    run_detection()
