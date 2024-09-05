import cv2
import utils
import time
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()


picam2.start()
time.sleep(2)

array = picam2.capture_array("main")
# # Variables to calculate FPS
counter, fps = 0, 0

width = 640
height = 480
# Visualization parameters
row_size = 20  # pixels
left_margin = 24  # pixels
text_color = (0, 0, 255)  # red
font_size = 1
font_thickness = 1
fps_avg_frame_count = 10

# Continuously capture images from the camera and run inference
while True:
    image = picam2.capture_array("main")

    counter += 1
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Draw keypoints and edges on input image
    image = utils.visualize(image,)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
        end_time = time.time()
        fps = fps_avg_frame_count / (end_time - start_time)
        start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
        break
    cv2.imshow('object_detector', image)

cv2.destroyAllWindows()