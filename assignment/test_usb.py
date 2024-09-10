import time
import board
import busio
from adafruit_mpu6050 import MPU6050

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MPU6050
mpu = MPU6050(i2c)

# Function to read accelerometer and gyroscope data
def read_mpu6050():
    accel = mpu.acceleration
    gyro = mpu.gyro
    return accel, gyro

# Main loop
while True:
    accel, gyro = read_mpu6050()
    print(f"Acceleration: {accel}")
    print(f"Gyroscope: {gyro}")
    time.sleep(1)
