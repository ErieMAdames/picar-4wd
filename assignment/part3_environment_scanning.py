import picar_4wd as pc4

speed = 30

def main():
    print('Starting Part 3: Environment Scanning')
    while True:
        scan_list = pc4.scan_step(35)
        if not scan_list:
            continue
        tmp = scan_list[3:7]
        print(scan_list)
        print(tmp)
        # if tmp != [2,2,2,2]:
        #     pc4.turn_right(speed)
        # else:
        #     pc4.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        pc4.stop()
