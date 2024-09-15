import picar_4wd as pc4
import time
while True:
    dist = input()
    start = time.time()
    pc4.forward(1)
    time.sleep(4.2 * (float(dist) / 100))
    stop = time.time()
    pc4.stop()
    print(stop - start)