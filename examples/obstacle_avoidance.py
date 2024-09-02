import picar_4wd as fc
import time
speed = 30

def main():
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue
        print('len(scan_list)')
        print(len(scan_list))
        tmp = scan_list[3:7]
        print(tmp)
        time.sleep(2)
        if tmp != [2,2,2,2]:
            fc.turn_right(speed)
        else:
            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()
    finally: 
        fc.stop()
