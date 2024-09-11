# # import time
# # from pyftdi.i2c import I2cController
# # from adafruit_mpu6050 import MPU6050

# # # Initialize FT200XD
# # i2c = I2cController()
# # i2c.configure('ftdi://ftdi:0403/6014/1')  # Adjust URL if needed

# # # Get I2C port for communication
# # i2c_port = i2c.get_port()

# # # Initialize MPU6050
# # mpu = MPU6050(i2c_port)

# # def read_angles():
# #     while True:
# #         # Read raw accelerometer and gyroscope values
# #         accel_x, accel_y, accel_z = mpu.acceleration
# #         gyro_x, gyro_y, gyro_z = mpu.gyro

# #         # Convert to angles if needed
# #         angle_x = accel_x  # Replace with actual conversion if necessary
# #         angle_y = accel_y
# #         angle_z = accel_z

# #         print(f"Angle X: {angle_x:.2f} degrees")
# #         print(f"Angle Y: {angle_y:.2f} degrees")
# #         print(f"Angle Z: {angle_z:.2f} degrees")

# #         time.sleep(1)

# # if __name__ == "__main__":
# #     read_angles()

# from pyftdi.i2c import I2cController
# from pyftdi.ftdi import Ftdi
# import time

# # Initialize I2C controller
# i2c = I2cController()

# # Use the correct URL based on your device details
# try:
#     # i2c.configure('ftdi://1027/24597/1')  # Ensure this URL is correct
#     url = 'ftdi://ftdi:0403/6015:D200B2IB/1'


#     i2c.configure(url)  # Ensure this URL is correct
#     # i2c.configure('ftdi:///1')  # Ensure this URL is correct
#     print("FTDI device configured successfully")
# except Exception as e:
#     print(f"Error configuring I2C: {e}")

# # List available devices
# for dev in Ftdi.list_devices():
#     print(f"Available device: {dev}")

#!/usr/bin/env python3

# Copyright (c) 2019-2024, Emmanuel Blot <emmanuel.blot@free.fr>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""List valid FTDI device URLs and descriptors."""

from argparse import ArgumentParser, FileType
from logging import Formatter, StreamHandler, DEBUG, ERROR
from sys import exit as sys_exit, modules, stderr
from traceback import format_exc
from pyftdi import FtdiLogger
from pyftdi.ftdi import Ftdi
from pyftdi.misc import add_custom_devices


def main():
    """Entry point."""
    debug = False
    try:
        argparser = ArgumentParser(description=modules[__name__].__doc__)
        argparser.add_argument('-P', '--vidpid', action='append',
                               help='specify a custom VID:PID device ID, '
                                    'may be repeated')
        argparser.add_argument('-V', '--virtual', type=FileType('r'),
                               help='use a virtual device, specified as YaML')
        argparser.add_argument('-v', '--verbose', action='count', default=0,
                               help='increase verbosity')
        argparser.add_argument('-d', '--debug', action='store_true',
                               help='enable debug mode')
        args = argparser.parse_args()
        debug = args.debug

        loglevel = max(DEBUG, ERROR - (10 * args.verbose))
        loglevel = min(ERROR, loglevel)
        if debug:
            formatter = Formatter('%(asctime)s.%(msecs)03d %(name)-20s '
                                  '%(message)s', '%H:%M:%S')
        else:
            formatter = Formatter('%(message)s')
        FtdiLogger.set_formatter(formatter)
        FtdiLogger.set_level(loglevel)
        FtdiLogger.log.addHandler(StreamHandler(stderr))

        if args.virtual:
            # pylint: disable=import-outside-toplevel
            from pyftdi.usbtools import UsbTools
            # Force PyUSB to use PyFtdi test framework for USB backends
            UsbTools.BACKENDS = ('pyftdi.tests.backend.usbvirt', )
            # Ensure the virtual backend can be found and is loaded
            backend = UsbTools.find_backend()
            loader = backend.create_loader()()
            loader.load(args.virtual)

        try:
            add_custom_devices(Ftdi, args.vidpid, force_hex=True)
        except ValueError as exc:
            argparser.error(str(exc))

        Ftdi.show_devices()

    except (ImportError, IOError, NotImplementedError, ValueError) as exc:
        print(f'\nError: {exc}', file=stderr)
        if debug:
            print(format_exc(chain=False), file=stderr)
        sys_exit(1)
    except KeyboardInterrupt:
        sys_exit(2)


if __name__ == '__main__':
    main()