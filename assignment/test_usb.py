# import time
# from pyftdi.i2c import I2cController
# from adafruit_mpu6050 import MPU6050

# # Initialize FT200XD
# i2c = I2cController()
# i2c.configure('ftdi://ftdi:0403/6014/1')  # Adjust URL if needed

# # Get I2C port for communication
# i2c_port = i2c.get_port()

# # Initialize MPU6050
# mpu = MPU6050(i2c_port)

# def read_angles():
#     while True:
#         # Read raw accelerometer and gyroscope values
#         accel_x, accel_y, accel_z = mpu.acceleration
#         gyro_x, gyro_y, gyro_z = mpu.gyro

#         # Convert to angles if needed
#         angle_x = accel_x  # Replace with actual conversion if necessary
#         angle_y = accel_y
#         angle_z = accel_z

#         print(f"Angle X: {angle_x:.2f} degrees")
#         print(f"Angle Y: {angle_y:.2f} degrees")
#         print(f"Angle Z: {angle_z:.2f} degrees")

#         time.sleep(1)

# if __name__ == "__main__":
#     read_angles()

from pyftdi.i2c import I2cController
from pyftdi.ftdi import Ftdi
import time

# Initialize I2C controller
i2c = I2cController()

# Use the correct URL based on your device details
try:
    # i2c.configure('ftdi://1027/24597/1')  # Ensure this URL is correct
    url = 'ftdi://ftdi:0403/6015:D200B2IB/1'


    i2c.configure(url)  # Ensure this URL is correct
    # i2c.configure('ftdi:///1')  # Ensure this URL is correct
    print("FTDI device configured successfully")
except Exception as e:
    print(f"Error configuring I2C: {e}")

# List available devices
for dev in Ftdi.list_devices():
    print(f"Available device: {dev}")
