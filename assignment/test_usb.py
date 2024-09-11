import time
import board
import busio
from adafruit_ft232h import FT232H

# Initialize the FT232H device
ft232h = FT232H()
i2c = busio.I2C(ft232h.get_i2c_by_index(0))

# Define the I2C address for your device
DEVICE_ADDRESS = 0x68  # Replace with your device's address

def read_data():
    try:
        # Replace with your specific I2C read commands
        data = i2c.readfrom(DEVICE_ADDRESS, 6)
        print("Data read from device:", data)
    except Exception as e:
        print("Error reading data:", e)

while True:
    read_data()
    time.sleep(1)
