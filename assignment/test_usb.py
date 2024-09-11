from pyftdi.i2c import I2cController

# Create an I2C controller instance
i2c = I2cController()

# Open the FTDI device
i2c.configure('ftdi://ftdi:232h/1')

# Example I2C communication
slave = i2c.get_port(0x68)  # Replace 0x68 with your I2C device address

# Read from the device
try:
    data = slave.read(10)  # Read 10 bytes
    print("Data read from device:", data)
except Exception as e:
    print("Error reading from I2C device:", e)
