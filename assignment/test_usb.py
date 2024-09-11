import time
from pyftdi.i2c import I2cController
from adafruit_mpu6050 import MPU6050

# Initialize FT200XD
i2c = I2cController()
i2c.configure('ftdi://ftdi:0403/6014/1')  # Adjust URL if needed

# Get I2C port for communication
i2c_port = i2c.get_port()

# Initialize MPU6050
mpu = MPU6050(i2c_port)

def read_angles():
    while True:
        # Read raw accelerometer and gyroscope values
        accel_x, accel_y, accel_z = mpu.acceleration
        gyro_x, gyro_y, gyro_z = mpu.gyro

        # Convert to angles if needed
        angle_x = accel_x  # Replace with actual conversion if necessary
        angle_y = accel_y
        angle_z = accel_z

        print(f"Angle X: {angle_x:.2f} degrees")
        print(f"Angle Y: {angle_y:.2f} degrees")
        print(f"Angle Z: {angle_z:.2f} degrees")

        time.sleep(1)

if __name__ == "__main__":
    read_angles()
