import numpy as np
import time
import sys
import cv2
from flask import Flask, Response
import threading
import picar_4wd as pc4
from heapq import heappop, heappush

# Initialize the Flask app
app = Flask(__name__)

# Set up global variables for streaming
frame = None

np.set_printoptions(threshold=sys.maxsize)

class SelfDrive:
    angle_offset = 0#-10

    # Setup GPIO
    def __init__(self):
        print('starting')
    def go_to(self, x, y):
        x_closest = min(x, 99)
        y_closest = min(y, 99)
        map = self.scan()
        temp_map = self.add_obstacle_buffer(map)
        path = self.a_star(temp_map, (0, 49), (y_closest, x_closest))
        if path:
            for p in path:
                map[p[0], p[1]] = 2
            self.create_frame(map)
            directions = {
                (1, 0): "up",
                (-1, 0): "down",
                (0, 1): "right",
                (0, -1): "left"
            }
            travel_instructions = []
            current_direction = None
            steps = 0
            for i in range(1, len(path)):
                prev_point = path[i - 1]
                current_point = path[i]
                new_direction = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
                if new_direction != current_direction:
                    if current_direction is not None:
                        direction_name = directions[current_direction]
                        travel_instructions.append((direction_name, steps))
                    current_direction = new_direction
                    steps = 1
                else:
                    steps += 1
            if current_direction is not None:
                direction_name = directions[current_direction]
                travel_instructions.append((direction_name, steps))
            prev_dir = ''
            direction = 'n'
            for t in travel_instructions:
                print('----------')
                print(direction)
                if t[0] == 'up':
                    if prev_dir == 'left':
                        self.turn_right()
                    if prev_dir == 'right':
                        self.turn_left()
                    prev_dir = 'up'
                    self.go_distance(t[1])
                elif t[0] == 'down':
                    if prev_dir == 'left':
                        self.turn_right()
                    if prev_dir == 'right':
                        self.turn_left()
                    prev_dir = 'down'
                    self.go_distance(-t[1])
                elif t[0] == 'left':
                    if direction == 'n':
                        direction = 'w'
                    elif direction == 'e':
                        direction = 'n'
                    elif direction == 'w':
                        direction = 's'
                    prev_dir = 'left'
                    self.turn_left()
                    self.go_distance(t[1])
                elif t[0] == 'right':
                    if direction == 'n':
                        direction = 'e'
                    elif direction == 'w':
                        direction = 'n'
                    elif direction == 'e':
                        direction = 's'
                    prev_dir = 'right'
                    self.turn_right()
                    self.go_distance(t[1])
                print(direction)
                print('-------')
            pc4.stop()
        if x != x_closest or y != y_closest:
            traveled_x = x_closest - 49
            traveled_y = y_closest
            new_dest_x = x - traveled_x
            new_dest_y = y - traveled_y
            if new_dest_x <= 2 and new_dest_y <= 2:
                self.go_to(new_dest_x, new_dest_y)
    def create_frame(self, map):
        global frame
        temp_map = self.add_obstacle_buffer(map)
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[temp_map == 0] = [34, 139, 34]  # Green for 0
        image[temp_map == 1] = [0,  36, 255]  # Red for 1
        image[temp_map == 2] = [255, 0,   0]  # Blue for path
        enlarged_image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_NEAREST)
        frame = cv2.flip(enlarged_image, 0)  # Flip the frame horizontally
    def scan(self):
        current_angle = -90
        granularity = 180
        us_step = int(180 / granularity)
        grid_size = 100
        map = np.zeros((grid_size, grid_size), dtype=int)
        for _ in range(granularity):
            distance = pc4.get_distance_at(current_angle)
            if distance > 0:
                dx = int(distance * np.cos(np.radians(current_angle + 90 + self.angle_offset))) + 49
                dy = int(distance * np.sin(np.radians(current_angle + 90 + self.angle_offset)))
                if 0 <= dx < 100 and 0 <= dy < 100:
                    map[dy, dx] = 1
            self.create_frame(map)
            current_angle += us_step
        return map
    def add_obstacle_buffer(self, grid, radius=10):
        rows, cols = grid.shape
        new_grid = np.copy(grid)
        obstacle_positions = np.argwhere(grid == 1)
        for obstacle in obstacle_positions:
            x, y = obstacle
            for i in range(max(0, x - radius), min(rows, x + radius + 1)):
                for j in range(max(0, y - radius), min(cols, y + radius + 1)):
                    if np.sqrt((x - i) ** 2 + (y - j) ** 2) <= radius:
                        new_grid[i, j] = 1

        return new_grid
    def a_star(self, grid, start, goal):
        # Heuristic: Manhattan distance (L1 norm)
        def heuristic(x1, y1, x2, y2):
            return abs(x1 - x2) + abs(y1 - y2)
        DIRECTIONS = {
            'up': (-1, 0),
            'down': (1, 0),
            'left': (0, -1),
            'right': (0, 1)
        }
        def get_neighbors(pos):
            neighbors = []
            for direction, (dr, dc) in DIRECTIONS.items():
                new_pos = (pos[0] + dr, pos[1] + dc)
                if 0 <= new_pos[0] < grid.shape[0] and 0 <= new_pos[1] < grid.shape[1] and grid[new_pos[0], new_pos[1]] == 0:
                    neighbors.append((new_pos, direction))
            return neighbors

        # A* Search
        open_set = []
        heappush(open_set, (0, start, None))  # (f_score, position, direction)
        
        came_from = {}  # To reconstruct the path
        g_score = {start: 0}  # Movement cost from start to current position
        f_score = {start: heuristic(start[0], start[1], goal[0], goal[1])}  # Total cost estimate (g + heuristic)

        while open_set:
            current_f_score, current, current_direction = heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current][0]
                path.append(start)
                return path[::-1]  # Return reversed path

            for neighbor, direction in get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if current_direction is not None and current_direction != direction:
                    tentative_g_score += 2  # Add penalty for turning (higher value = stronger preference for straight)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = (current, direction)
                    g_score[neighbor] = tentative_g_score
                    # Add a small bias in the heuristic to prefer straight-line movement
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor[0], neighbor[1], goal[0], goal[1])
                    heappush(open_set, (f_score[neighbor], neighbor, direction))
        return None  # No path found

    def turn_right(self):
        print('turnging right')
        pc4.turn_right(1000)
        input()
        pc4.stop()

    def turn_left(self):
        print('turnging left')
        pc4.turn_left(1000)
        input()
        pc4.stop()
    def go_distance(self, dist):
        if dist > 0:
            pc4.forward(1)
        else:
            pc4.backward(1)
        print('forward dist: ' + str(dist) + ' cm | time ' + str(abs(dist/100) * 4.2) + ' seconds')
        time.sleep(abs(dist/100) * 4.2)
        pc4.stop()
        time.sleep(.5)
def generate_frames():
    global frame
    while True:
        if frame is not None:
            ret, png = cv2.imencode('.png', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/png\r\n\r\n' + png.tobytes() + b'\r\n')
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
        # Create a SelfDrive object
        drive = SelfDrive()
        # Create a thread for the Flask server
        flask_thread = threading.Thread(target=run_flask)
        flask_thread.start()

        # Run the scan method in the main thread
        while True:
            drive.go_to(150, 150)
            input()

    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()