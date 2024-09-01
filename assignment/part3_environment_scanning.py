import picar_4wd as fc

speed = 30

def main():
    print('Starting Part 3: Environment Scanning')
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
        tmp = scan_list[3:7]
        print(scan_list)
        print(tmp)
        if tmp != [2,2,2,2]:
            fc.turn_right(speed)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
