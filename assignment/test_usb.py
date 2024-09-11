from mpu6050 import mpu6050
import time

# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)  # Default I2C address for MPU-6050 is 0x68

# Variables to store the angle
turning_angle = 0.0  # Initial angle in degrees
prev_time = time.time()

def read_gyro_data():
    """Reads the gyroscope data from the IMU sensor."""
    gyro_data = sensor.get_gyro_data()
    return gyro_data

def calculate_turning_angle(gyro_z, dt):
    """Calculates the turning angle from gyroscope data."""
    global turning_angle
    # Integrate angular velocity (in degrees per second) over time (in seconds)
    turning_angle += gyro_z * dt
    return turning_angle

def main():
    global prev_time

    try:
        while True:
            # Get current time
            current_time = time.time()
            dt = current_time - prev_time  # Time difference
            prev_time = current_time

            # Read gyroscope data
            gyro_data = read_gyro_data()
            gyro_z = gyro_data['z']  # Z-axis represents yaw rotation

            # Calculate current turning angle
            angle = calculate_turning_angle(gyro_z, dt)
            print(f"Current Turning Angle: {angle:.2f} degrees")

            time.sleep(0.1)  # Delay to reduce noise and limit data rate

    except KeyboardInterrupt:
        print("Stopping IMU data acquisition...")

if __name__ == "__main__":
    main()
