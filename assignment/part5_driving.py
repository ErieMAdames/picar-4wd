import picar_4wd as pc4
import RPi.GPIO as GPIO
import time
from functools import reduce

speed = 60
# turning_time = .9
# current_angle = 0
# us_step = pc4.STEP
# min_angle = -36
# max_angle = 36
# distances = []

# def scan():
#     global distances, current_angle, us_step
#     current_angle = 90 if current_angle > 0 else -90
#     us_step = -pc4.STEP if current_angle > 0 else pc4.STEP
#     for _ in range(5):
#         current_angle += us_step
#         if current_angle >= max_angle:
#             current_angle = max_angle
#             us_step = -pc4.STEP
#         elif current_angle <= min_angle:
#             current_angle = min_angle
#             us_step = pc4.STEP
#         distance = pc4.get_distance_at(current_angle)
#         distances.append(distance)
#     if us_step < 0:
#         distances.reverse()
#     distances_map = map(lambda x: x < 30 and x != -2, distances)
#     stop = reduce(lambda x, y: x or y, distances_map)
#     distances = []
#     return stop

# def avoid_left():
#     # if there is something in the way while avoiding, retrace stepts
#     retrace_steps = []
#     pc4.turn_left(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         # turn right to retrace
#         retrace_steps.append('r')
#         return retrace_steps
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     pc4.turn_right(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         retrace_steps.append('b')
#         retrace_steps.append('l')
#         return retrace_steps
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     pc4.turn_right(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         retrace_steps.append('b')
#         retrace_steps.append('l')
#         return retrace_steps
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.turn_left(speed)
#     time.sleep(turning_time)
#     stop = scan()
#     if stop:
#         retrace_steps.append('b')
#         retrace_steps.append('r')
#         return retrace_steps
#     pc4.forward(speed)
#     return retrace_steps
# def avoid_right():
#     pc4.turn_right(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         return True
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     pc4.turn_left(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         return True
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     pc4.turn_left(speed)
#     time.sleep(turning_time)
#     pc4.stop()
#     stop = scan()
#     if stop:
#         return True
#     pc4.forward(speed)
#     time.sleep(turning_time)
#     pc4.turn_right(speed)
#     time.sleep(turning_time)
#     stop = scan()
#     if stop:
#         return True
#     pc4.forward(speed)
#     return False
# def retrace(retrace_steps):
#     for step in reversed(retrace_steps):
#         if step == 'l':
#             pc4.turn_left(speed)
#             time.sleep(turning_time)
#         if step == 'r':
#             pc4.turn_right(speed)
#             time.sleep(turning_time)
#         if step == 'b':
#             pc4.backward(speed)
#             time.sleep(turning_time)

# Define GPIO pins for photointerruptors
LEFT_ENCODER_PIN = 25  # Replace with your GPIO pin number
RIGHT_ENCODER_PIN = 4  # Replace with your GPIO pin number

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(RIGHT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LEFT_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to store encoder counts
left_encoder_count = 0
right_encoder_count = 0
# Callback functions to increment counts
def left_encoder_callback(channel):
    global left_encoder_count
    left_encoder_count += 1

def right_encoder_callback(channel): 
    global right_encoder_count
    right_encoder_count += 1

# # Add event detection for rising edges
GPIO.add_event_detect(LEFT_ENCODER_PIN, GPIO.RISING, callback=left_encoder_callback)
GPIO.add_event_detect(RIGHT_ENCODER_PIN, GPIO.RISING, callback=right_encoder_callback)
# def go_distance(dist, forward=True):
#     global left_encoder_count, right_encoder_count
#     left_encoder_count = 0
#     right_encoder_count = 0
#     WHEEL_DIAMETER = 0.0662  # Example wheel diameter in meters
#     PPR = 20  # Example pulses per revolution
#     def calculate_distance(counts):
#         wheel_circumference = WHEEL_DIAMETER * 3.14159
#         distance = (counts / PPR) * wheel_circumference
#         return distance

#     left_distance = calculate_distance(left_encoder_count)
#     right_distance = calculate_distance(right_encoder_count)
#     while left_distance < dist or right_distance < dist:
#         if forward:
#             pc4.forward(speed)
#         else:
#             pc4.backward(speed)
#         left_distance = calculate_distance(left_encoder_count)
#         right_distance = calculate_distance(right_encoder_count)
#     pc4.stop()
    
#     print(f"Left Encoder Count: {left_encoder_count}, Right Encoder Count: {right_encoder_count}")
#     print(f"Left Distance: {left_distance}, Right Distance: {right_distance}")

# try:
#     x = input()
#     go_distance(float(x))
#     go_distance(float(x), False)

# except KeyboardInterrupt:
#     print("Program stopped by user")
# finally:
#     GPIO.cleanup()  # Clean up GPIO on exit
def calculate_distance(counts):
        WHEEL_DIAMETER = 0.0662  # Example wheel diameter in meters
        PPR = 20  # Example pulses per revolution
        wheel_circumference = WHEEL_DIAMETER * 3.14159
        distance = (counts / PPR) * wheel_circumference
        return distance

def main(right=True):
    global distances, current_angle, us_step
    speed = 50
    try:
        while True:
            if right:
                pc4.turn_right(speed)  # Start turning right
            else:
                pc4.turn_left(speed)  # Start turning right
            time.sleep(0.1)  # Adjust the sleep time to prevent excessive CPU usage
    except KeyboardInterrupt:
        pc4.stop()
        end_time = time.time()

        # Calculate and display the time taken
        duration = end_time - start_time
        print(f"Time taken for turn: {duration:.2f} seconds")

        # Calculate the time required for a 90-degree turn
        # Adjust this based on your own measurements
        turn_duration_for_90_degrees = duration  # Replace with actual calibration value
        if right:
            print(f"Estimated duration for 90-degree right turn: {turn_duration_for_90_degrees:.2f} seconds")
        else:
            print(f"Estimated duration for 90-degree left turn: {turn_duration_for_90_degrees:.2f} seconds")
        left_distance = calculate_distance(left_encoder_count)
        right_distance = calculate_distance(right_encoder_count)
        
        print(f"Left Encoder Count: {left_encoder_count}, Right Encoder Count: {right_encoder_count}")
        print(f"Left Distance: {left_distance}, Right Distance: {right_distance}")

if __name__ == "__main__":
    start_time = time.time()
    try:
        print('Starting Part 5: Move around object')
        main(True)
        main(False)
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()