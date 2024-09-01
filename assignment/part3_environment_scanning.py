import picar_4wd as pc4
from functools import reduce
speed = 15

def main():
    print('Starting Part 3: Environment Scanning')
    current_angle = 0
    us_step = pc4.STEP
    distances = []
    while True:
        # scan_list = pc4.scan_step(35)
        current_angle += us_step
        if current_angle >= pc4.max_angle:
            current_angle = pc4.max_angle
            us_step = -pc4.STEP
        elif current_angle <= pc4.min_angle:
            current_angle = pc4.min_angle
            us_step = pc4.STEP
        distance = pc4.get_distance_at(current_angle)
        # print(str(current_angle) + ' | ' + str(distance))
        distances.append(distance)
        distances_map = map(lambda x: x < 15 and x != -2)
        stop = reduce(lambda x, y: x and y, distances_map)
        print(distances)
        print(stop)
        # if distance < 15:
        #     pc4.stop()
        # else:
        #     pc4.forward(speed)
        if current_angle == pc4.min_angle or current_angle == pc4.max_angle:
            distances = []


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print('Stopping')
    finally:
        pc4.stop()
