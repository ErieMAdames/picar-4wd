import picar_4wd as pc4
import time
from functools import reduce

speed = 30
turning_time = .9
current_angle = 0
us_step = pc4.STEP
min_angle = -36
max_angle = 36
distances = []

def scan():
    global distances, current_angle, us_step
    current_angle = 90 if current_angle > 0 else -90
    us_step = -pc4.STEP if current_angle > 0 else pc4.STEP
    for _ in range(5):
        current_angle += us_step
        if current_angle >= max_angle:
            current_angle = max_angle
            us_step = -pc4.STEP
        elif current_angle <= min_angle:
            current_angle = min_angle
            us_step = pc4.STEP
        distance = pc4.get_distance_at(current_angle)
        distances.append(distance)
    if us_step < 0:
        distances.reverse()
    distances_map = map(lambda x: x < 30 and x != -2, distances)
    stop = reduce(lambda x, y: x or y, distances_map)
    distances = []
    return stop

def avoid_left():
    # if there is something in the way while avoiding, retrace stepts
    retrace_steps = []
    pc4.turn_left(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        # turn right to retrace
        retrace_steps.append('r')
        return retrace_steps
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.stop()
    pc4.turn_right(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        retrace_steps.append('b')
        retrace_steps.append('l')
        return retrace_steps
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.stop()
    pc4.turn_right(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        retrace_steps.append('b')
        retrace_steps.append('l')
        return retrace_steps
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.turn_left(speed)
    time.sleep(turning_time)
    stop = scan()
    if stop:
        retrace_steps.append('b')
        retrace_steps.append('r')
        return retrace_steps
    pc4.forward(speed)
    return retrace_steps
def avoid_right():
    pc4.turn_right(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        return True
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.stop()
    pc4.turn_left(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        return True
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.stop()
    pc4.turn_left(speed)
    time.sleep(turning_time)
    pc4.stop()
    stop = scan()
    if stop:
        return True
    pc4.forward(speed)
    time.sleep(turning_time)
    pc4.turn_right(speed)
    time.sleep(turning_time)
    stop = scan()
    if stop:
        return True
    pc4.forward(speed)
    return False
def retrace(retrace_steps):
    for step in reversed(retrace_steps):
        if step == 'l':
            pc4.turn_left(speed)
            time.sleep(turning_time)
        if step == 'r':
            pc4.turn_right(speed)
            time.sleep(turning_time)
        if step == 'b':
            pc4.backward(speed)
            time.sleep(turning_time)

def main():
    global distances, current_angle, us_step
    while True:
        x = input()
        s = x.split(',')
        speed = float(s[0])
        turning_time = float(s[1])
        
        pc4.left_front.set_power(-speed)
        pc4.left_rear.set_power(-speed)
        # pc4.right_front.set_power(power)
        # pc4.right_rear.set_power(power)
        # pc4.turn_left(speed)
        time.sleep(turning_time)
        pc4.stop()
        # stop = scan()
        # if stop:
        #     pc4.stop()
        #     retrace_steps = avoid_left()
        #     retrace(retrace_steps)
        #     if len(retrace_steps) > 0:
        #         stop = avoid_right()
        #         if stop:
        #             print('no path')
        #             break

        # else:
        #     pc4.forward(speed)

if __name__ == "__main__":
    try:
        print('Starting Part 5: Move around object')
        main()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
