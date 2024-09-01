import picar_4wd as pc4
import time
from functools import reduce

speed = 25
current_angle = 0
us_step = pc4.STEP
distances = []

def scan():
    global distances, current_angle, us_step
    current_angle = 90 if current_angle > 0 else -90
    us_step = -pc4.STEP if current_angle > 0 else pc4.STEP
    for _ in range(10):
        current_angle += us_step
        if current_angle >= pc4.max_angle:
            current_angle = pc4.max_angle
            us_step = -pc4.STEP
        elif current_angle <= pc4.min_angle:
            current_angle = pc4.min_angle
            us_step = pc4.STEP
        distance = pc4.get_distance_at(current_angle)
        distances.append(distance)
    distances_map = map(lambda x: x < 35 and x != -2, distances[3:7])
    stop = reduce(lambda x, y: x or y, distances_map)
    distances = []
    return stop

def avoid():
    pc4.turn_left(speed)
    time.sleep(.5)
    pc4.stop()
    if not scan():
        pc4.forward(speed)
        time.sleep(.5)
        pc4.stop()
        pc4.turn_right(speed)
        time.sleep(.5)
        pc4.stop()
        if not scan():
            pc4.forward(speed)
            time.sleep(.5)
            pc4.stop()
            pc4.turn_right(speed)
            time.sleep(.5)
            pc4.stop()
            if not scan():
                pc4.forward(speed)
                time.sleep(.5)
                pc4.turn_right(speed)
                time.sleep(.5)
                if not scan():
                    pc4.forward(speed)
    else:
        pc4.turn_right(speed)
        time.sleep(1)
        pc4.stop()
        if not scan():
            pc4.forward(speed)
            time.sleep(.5)
            pc4.stop()
            pc4.turn_left(speed)
            time.sleep(.5)
            pc4.stop()
            if not scan():
                pc4.forward(speed)
                time.sleep(.5)
                pc4.stop()
                pc4.turn_left(speed)
                time.sleep(.5)
                pc4.stop()
                if not scan():
                    pc4.forward(speed)
                    time.sleep(.5)
                    pc4.turn_left(speed)
                    time.sleep(.5)
                    if not scan():
                        pc4.forward(speed)


    

def main():
    global distances, current_angle, us_step
    while True:
        current_angle += us_step
        if current_angle >= pc4.max_angle:
            current_angle = pc4.max_angle
            us_step = -pc4.STEP
        elif current_angle <= pc4.min_angle:
            current_angle = pc4.min_angle
            us_step = pc4.STEP
        distance = pc4.get_distance_at(current_angle)
        distances.append(distance)
        if len(distances) == 10:
            distances_map = map(lambda x: x < 35 and x != -2, distances[3:7])
            stop = reduce(lambda x, y: x or y, distances_map)
            if stop:
                pc4.stop()
                avoid()
            else:
                pc4.forward(speed)
        if current_angle == pc4.min_angle or current_angle == pc4.max_angle:
            distances = []


if __name__ == "__main__":
    try:
        print('Starting Part 3: Move around object')
        main()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
