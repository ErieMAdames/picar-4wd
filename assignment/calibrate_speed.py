import picar_4wd as pc4
import time

start = time.time()
pc4.forward(1)
time.sleep(4.2 * (float(input()) / 100))
stop = time.time()
pc4.stop()
print(stop - start)