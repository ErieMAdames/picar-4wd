import time
import board
import busio
import adafruit_mpu6050

# Initialize I2C using FT232H
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize MPU6050
mpu = adafruit_mpu6050.MPU6050(i2c)

# Function to read angles
def read_angles():
    accel = mpu.acceleration
    gyro = mpu.gyro
    print(f"Acceleration: X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f}")
    print(f"Gyro: X={gyro[0]:.2f}, Y={gyro[1]:.2f}, Z={gyro[2]:.2f}")

# Read angles in a loop
while True:
    read_angles()
    time.sleep(0.5)
