import picar_4wd as pc4
import time
from functools import reduce
import random

speed = 25

def main():
    print('Starting Part 3: Environment Scanning')
    current_angle = 0
    us_step = pc4.STEP
    distances = []
    reversed = False
    left = True
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
                pc4.backward(speed)
                    time.sleep(.5)
                if not reversed:
                    reversed = True
                    left = random.random() > .5
            else:
                if reversed:
                    if left:
                        pc4.turn_left(speed)
                    else:
                        pc4.turn_right(speed)
                    time.sleep(.5)
                    reversed = False
                else:
                    pc4.forward(speed)
        if current_angle == pc4.min_angle or current_angle == pc4.max_angle:
            distances = []


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
