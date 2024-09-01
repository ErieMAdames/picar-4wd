import picar_4wd as pc4

speed = 30

def main():
    print('Starting Part 3: Environment Scanning')
    current_angle = 0
    us_step = pc4.STEP
    scan_list = []
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
        print(str(current_angle) + ' | ' + str(distance))
        # print(str(pc4.current_angle) + ' | ' + str(pc4.get_distance_at(pc4.current_angle)))
        # if not scan_list:
        #     continue
        # tmp = scan_list[3:7]
        # print(scan_list)
        # print(tmp)
        # if tmp != [2,2,2,2]:
        #     pc4.turn_right(speed)
        # else:
        #     pc4.forward(speed)


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print('Stopping')
    finally:
        pc4.stop()
