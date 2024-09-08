from mpu6050 import mpu6050
import time
imu = mpu6050(0x68)
x = 0
z = 0
y = 0
counter = 0
try:
    while True:
        g = imu.get_gyro_data()
        x += g['x']
        y += g['y']
        z += g['z']
        counter += 1
except KeyboardInterrupt:
    print('X ' + str(x / counter))
    print('Y ' + str(y / counter))
    print('Z ' + str(z / counter))