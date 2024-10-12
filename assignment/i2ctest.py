import time
from picar_4wd.i2c import I2C

class ICM20948:
    ICM20948_ADDRESS = 0x68  # ICM20948 I2C address
    MAG_ADDRESS = 0x0C  # Magnetometer I2C address (AK09916)

    WHO_AM_I = 0x00  # Register to check the ICM20948 identity
    ACCEL_XOUT_H = 0x2D  # Starting address for accelerometer data
    GYRO_XOUT_H = 0x33   # Starting address for gyroscope data
    PWR_MGMT_1 = 0x06    # Power management register
    
    # Magnetometer registers
    MAG_WIA2 = 0x01  # Who am I register for the magnetometer
    MAG_ST1 = 0x10   # Status 1 register (data ready)
    MAG_XOUT_L = 0x11  # Magnetometer data start register (low byte X)
    MAG_CNTL2 = 0x31  # Control register 2 (to trigger single measurement mode)

    USER_CTRL = 0x03  # User control to enable I2C master mode
    I2C_MST_CTRL = 0x24  # I2C master control register
    I2C_SLV0_ADDR = 0x25  # I2C slave 0 address (magnetometer address)
    I2C_SLV0_REG = 0x26  # I2C slave 0 register (magnetometer data register)
    I2C_SLV0_CTRL = 0x27  # I2C slave 0 control register
    EXT_SENS_DATA_00 = 0x3B  # External sensor data (where magnetometer data will appear)
    
    def __init__(self):
        self.i2c = I2C(self.ICM20948_ADDRESS)
        self.initialize_sensor()
        self.initialize_magnetometer()

    def initialize_sensor(self):
        # Wake up the sensor (write 0x01 to PWR_MGMT_1)
        self.i2c.write_reg(self.PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # Enable I2C master mode
        self.i2c.write_reg(self.USER_CTRL, 0x20)  # Enable I2C master
        self.i2c.write_reg(self.I2C_MST_CTRL, 0x0D)  # Set I2C master clock to 345.6 kHz

    def initialize_magnetometer(self):
        # Setup magnetometer for continuous measurement mode
        self.i2c.write_reg(self.I2C_SLV0_ADDR, self.MAG_ADDRESS | 0x80)  # Set slave 0 address to magnetometer (read mode)
        self.i2c.write_reg(self.I2C_SLV0_REG, self.MAG_CNTL2)  # Set slave 0 register to MAG_CNTL2 (control 2)
        self.i2c.write_reg(self.I2C_SLV0_CTRL, 0x81)  # Enable reading 1 byte from slave 0
        self.i2c.write_reg(self.I2C_SLV0_REG, self.MAG_WIA2)  # Check who am I for magnetometer (WIA2)
        mag_who_am_i = self.i2c.read(self.EXT_SENS_DATA_00, 1)
        if mag_who_am_i[0] != 0x09:
            raise Exception("Magnetometer not detected.")
        # Set magnetometer to continuous measurement mode (0x08)
        self.i2c.write_reg(self.I2C_SLV0_REG, self.MAG_CNTL2)
        self.i2c.write_reg(self.I2C_SLV0_CTRL, 0x81)  # Write 1 byte to control register
        self.i2c.write_reg(self.EXT_SENS_DATA_00, 0x08)
    
    def read_accel_data(self):
        accel_data = self.i2c.read(self.ACCEL_XOUT_H, 6)
        accel_x = (accel_data[0] << 8) | accel_data[1]
        accel_y = (accel_data[2] << 8) | accel_data[3]
        accel_z = (accel_data[4] << 8) | accel_data[5]
        return {'x': accel_x, 'y': accel_y, 'z': accel_z}

    def read_gyro_data(self):
        gyro_data = self.i2c.read(self.GYRO_XOUT_H, 6)
        gyro_x = (gyro_data[0] << 8) | gyro_data[1]
        gyro_y = (gyro_data[2] << 8) | gyro_data[3]
        gyro_z = (gyro_data[4] << 8) | gyro_data[5]
        return {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}

    def read_magnetometer_data(self):
        # Read magnetometer data (6 bytes starting from MAG_XOUT_L)
        self.i2c.write_reg(self.I2C_SLV0_ADDR, self.MAG_ADDRESS | 0x80)  # Set slave to magnetometer (read mode)
        self.i2c.write_reg(self.I2C_SLV0_REG, self.MAG_XOUT_L)  # Set register to magnetometer data start
        self.i2c.write_reg(self.I2C_SLV0_CTRL, 0x87)  # Enable reading 7 bytes from slave 0
        
        mag_data = self.i2c.read(self.EXT_SENS_DATA_00, 7)  # Read 7 bytes from EXT_SENS_DATA_00
        if mag_data[0] & 0x01 == 0:  # Check if data is ready
            return None

        mag_x = (mag_data[1] << 8) | mag_data[2]
        mag_y = (mag_data[3] << 8) | mag_data[4]
        mag_z = (mag_data[5] << 8) | mag_data[6]
        return {'x': mag_x, 'y': mag_y, 'z': mag_z}

    def who_am_i(self):
        who_am_i = self.i2c.read(self.WHO_AM_I, 1)
        return who_am_i[0]

# Example usage
if __name__ == "__main__":
    imu = ICM20948()
    print("WHO_AM_I:", hex(imu.who_am_i()))
    while True:
        accel = imu.read_accel_data()
        gyro = imu.read_gyro_data()
        mag = imu.read_magnetometer_data()
        print(f"Accel: {accel}, Gyro: {gyro}, Mag: {mag}")
        time.sleep(1)