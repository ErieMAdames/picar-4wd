import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
import json
import math
from functools import reduce
import threading
import asyncio
import websockets
import sys
class AvoidObjects():
    current_car_angle = 0
    speed = 30
    turning_time = .9
    current_angle = 0
    us_step = pc4.STEP
    min_angle = -36
    max_angle = 36
    distances = []
    WHEEL_DIAMETER = 0.0662  # Example wheel diameter in meters
    PPR = 20  # Example pulses per revolution
    LEFT_ENCODER_PIN = 25  # Replace with your GPIO pin number
    RIGHT_ENCODER_PIN = 4  # Replace with your GPIO pin number
    left_encoder_count = 0
    right_encoder_count = 0
    WEBSOCKET_URL = "ws://192.168.86.246:8080/sensor/connect?type=android.sensor.rotation_vector"
    # Setup GPIO
    def __init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.LEFT_ENCODER_PIN, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_PIN, GPIO.RISING, callback=self.right_encoder_callback)
        client_thread = threading.Thread(target=self.start_websocket_client)
        client_thread.start()
        time.sleep(2)
        print('starting')
        self.turn()
        sys.exit(0)
        while True:
            traveled = self.go_distance(10, True)
            if traveled < 10:
                retrace_steps = self.avoid()
                if len(retrace_steps):
                    self.retrace(retrace_steps)
                    retrace_steps = self.avoid(False)
                    if len(retrace_steps):
                        print('No path')
                        sys.exit(0)
    # Variables to store encoder counts
    # Callback functions to increment counts
    def left_encoder_callback(self, channel):
        self.left_encoder_count += 1

    def right_encoder_callback(self, channel): 
        self.right_encoder_count += 1

    # # Add event detection for rising edges
    def scan(self):
        self.current_angle = 90 if self.current_angle > 0 else -90
        self.us_step = -pc4.STEP if self.current_angle > 0 else pc4.STEP
        for _ in range(5):
            self.current_angle += self.us_step
            if self.current_angle >= self.max_angle:
                self.current_angle = self.max_angle
                self.us_step = -pc4.STEP
            elif self.current_angle <= self.min_angle:
                self.current_angle = self.min_angle
                self.us_step = pc4.STEP
            distance = pc4.get_distance_at(self.current_angle)
            self.distances.append(distance)
        if self.us_step < 0:
            self.distances.reverse()
        distances_map = map(lambda x: x < 30 and x != -2, self.distances)
        stop = reduce(lambda x, y: x or y, distances_map)
        self.distances = []
        return stop

    def avoid(self, right=True):
        # if there is something in the way while avoiding, retrace stepts
        retrace_steps = []
        self.turn(right, 90, self.speed)
        stop = self.scan()
        if stop:
            # turn right to retrace
            retrace_steps.append((not right,0))
            return retrace_steps
        dist = self.go_distance(.4, True)
        if dist is not None:
            retrace_steps.append(('b', dist))
            return retrace_steps
        self.turn(not right, 90, self.speed)
        stop = self.scan()
        if stop:
            retrace_steps.append(('b', dist))
            retrace_steps.append((right, 0))
            return retrace_steps
        dist = self.go_distance(.4, True)
        if dist is not None:
            retrace_steps.append(('b', dist))
            return retrace_steps
        self.turn(not right, 90, self.speed)
        stop = self.scan()
        if stop:
            retrace_steps.append(('b', dist))
            retrace_steps.append((right, 0))
            return retrace_steps
        dist = self.go_distance(.4, True)
        if dist is not None:
            retrace_steps.append(('b', dist))
            return retrace_steps
        self.turn(right, 90, self.speed)
        stop = self.scan()
        if stop:
            retrace_steps.append(('b', dist))
            retrace_steps.append((not right, 0))
            return retrace_steps
        dist = self.go_distance(.4, True)
        if dist is not None:
            retrace_steps.append(('b', dist))
            return retrace_steps
        return retrace_steps
    def retrace(self, retrace_steps):
        for step in reversed(retrace_steps):
            if step[0] == 'b':
                self.go_distance(step[1], False)
            else:
                self.turn(step[0], 90, self.speed)

    def go_distance(self, dist, forward=True):
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        def calculate_distance(counts):
            wheel_circumference = self.WHEEL_DIAMETER * 3.14159
            distance = (counts / self.PPR) * wheel_circumference
            return distance

        left_distance = calculate_distance(self.left_encoder_count)
        right_distance = calculate_distance(self.right_encoder_count)
        
        while left_distance < dist or right_distance < dist:
            stop = self.scan()
            if stop:
                break
            if forward:
                pc4.forward(self.speed)
            else:
                pc4.backward(self.speed)
            left_distance = calculate_distance(self.left_encoder_count)
            right_distance = calculate_distance(self.right_encoder_count)
        pc4.stop()
        return (left_distance + right_distance) / 2

    def turn(self, right=True, angle=90, speed=30):
        start_angle = self.current_car_angle
        a = self.current_car_angle - start_angle
        a = abs((a + 180) % 360 - 180)
        if right:
            pc4.turn_right(speed)
        else:
            pc4.turn_left(speed)
        speed_lowered = False
        while a < angle:
            a = self.current_car_angle - start_angle
            a = abs((a + 180) % 360 - 180)
            error = abs((a - angle)/angle)
            if error < .25 and not speed_lowered:
                print('lowering speed')
                speed_lowered = True
                if right:
                    pc4.turn_right(speed/2)
                else:
                    pc4.turn_left(speed/2)
        # print('----')
        # print(start_angle)
        # print(self.current_car_angle)
        # print(a)
        # print(a/angle)
        # print(a - angle)
        # print('----')
        if abs((a - angle)/angle) > .1:
            self.turn(not right, a - angle, speed/2)


    def get_orientation_from_rotation_vector(self, rotation_vector):
        x = rotation_vector[0]
        y = rotation_vector[1]
        z = rotation_vector[2]
        w = rotation_vector[3] if len(rotation_vector) > 3 else 1.0

        # Convert quaternion to Euler angles
        yaw = math.atan2(2.0 * (x * y + z * w), 1.0 - 2.0 * (y * y + z * z))
        # pitch = math.asin(2.0 * (x * z - w * y))
        # roll = math.atan2(2.0 * (x * w + y * z), 1.0 - 2.0 * (z * z + w * w))

        # Convert radians to degrees
        yaw = math.degrees(yaw)
        # pitch = math.degrees(pitch)
        # roll = math.degrees(roll)

        return yaw #, pitch, roll
    async def receive_data(self):
        # Connect to the WebSocket server
        async with websockets.connect(self.WEBSOCKET_URL) as websocket:
            print(f"Connected to {self.WEBSOCKET_URL}")
            try:
                while True:
                    # Receive data from the server
                    data = await websocket.recv()
                    data = json.loads(data)
                    angle = self.get_orientation_from_rotation_vector(data['values'])
                    if angle < 0:
                        angle = angle + 360
                    self.current_car_angle = angle

            except websockets.ConnectionClosed as e:
                print(f"Connection closed: {e}")
                await self.receive_data()

    def start_websocket_client(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.receive_data())

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')
        AvoidObjects()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
        GPIO.cleanup()  # Clean up GPIO on exit