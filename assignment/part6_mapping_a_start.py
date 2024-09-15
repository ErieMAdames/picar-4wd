import numpy as np
import time
import sys
import cv2
from flask import Flask, Response
import threading
import RPi.GPIO as GPIO
import picar_4wd as pc4
from heapq import heappop, heappush

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
    angle_offset = -10

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
        global frame
        print('scanning')
        self.current_angle = -90
        self.us_step = 1
        grid_size = 100
        map_grid = np.zeros((grid_size, grid_size), dtype=int)
        for _ in range(180):
            distance = pc4.get_distance_at(self.current_angle)
            if distance > 0:
                dx = int(distance * np.cos(np.radians(self.current_angle + 90 + self.angle_offset))) + 49
                dy = int(distance * np.sin(np.radians(self.current_angle + 90 + self.angle_offset)))
                
                if 0 <= dx < 100 and 0 <= dy < 100:
                    map_grid[dy, dx] = 1

                # Create and process the image with OpenCV
                image = np.zeros((100, 100, 3), dtype=np.uint8)
                image[map_grid == 0] = [0, 255, 0]  # Green for 0
                image[map_grid == 1] = [0, 0, 255]  # Red for 1

                enlarged_image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_NEAREST)
                # rotated_image = cv2.rotate(enlarged_image, cv2.ROTATE_90_CLOCKWISE)

                # Prepare the frame for streaming
                frame = cv2.flip(enlarged_image, 0)  # Flip the frame horizontally
            self.current_angle += self.us_step
        
        for x in np.flip(map_grid, 0):
            x_str = np.array_repr(x).replace('\n', '').replace(' ', '').replace('array([', '').replace('])', '').replace('0','_').replace('1','@')
            print(x_str)
        self.distances = []
        print(self.a_star(map_grid, (49, 0), (99, 99)))
    def a_star(self, grid, start, goal):
        # Heuristic: Manhattan distance (L1 norm)
        def heuristic(start_coords, goal_coords):
            x1, y1 = start_coords
            x2, y2 = goal_coords
            return abs(x1 - x2) + abs(y1 - y2)

        # Valid neighbors (up, down, left, right)
        def get_neighbors(pos):
            neighbors = [
                (pos[0] - 1, pos[1]),  # Up
                (pos[0] + 1, pos[1]),  # Down
                (pos[0], pos[1] - 1),  # Left
                (pos[0], pos[1] + 1)   # Right
            ]
            # Filter neighbors to be within bounds and not obstacles
            valid_neighbors = [(r, c) for r, c in neighbors if 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1] and grid[r, c] == 0]
            return valid_neighbors

        # A* Search
        open_set = []
        heappush(open_set, (0, start))  # (f_score, position)
        
        came_from = {}  # To reconstruct the path
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_set:
            current = heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)  # Add the start point
                return path[::-1]  # Return reversed path
            
            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + 1  # Each move costs 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found
    

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
        while True:
            map_instance.scan()
            input()

    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()
