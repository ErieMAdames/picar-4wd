import cv2
import time
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
import threading

# Initialize the Flask app
app = Flask(__name__)
frame = None

def run_detection():
    global frame
    # Start the camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
    picam2.start()

    time.sleep(2)  # Allow the camera to warm up

    while True:
        # Capture frame-by-frame
        image = picam2.capture_array("main")
        image = cv2.flip(image, 1)  # Flip image for correct orientation

        # Encode the frame in JPEG format
        ret, jpeg = cv2.imencode('.jpg', image)
        if ret:
            frame = jpeg.tobytes()  # Store the JPEG frame

        time.sleep(0.1)  # Control frame rate

def generate_frames():
    global frame
    while True:
        if frame is not None:
            # Yield the frame to the web page
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    # Route for video streaming
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    # Start Flask server
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == "__main__":
    # Start Flask server in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.start()

    # Run the detection and frame capture function
    run_detection()
