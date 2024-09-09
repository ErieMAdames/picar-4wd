#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading

#     time.sleep(1)
import RPi.GPIO as GPIO

import smbus, math
from .i2c import I2C

import time
# from picar_4wd.servo import Servo
# from picar_4wd.pwm import PWM
# from picar_4wd.pin import Pin


import subprocess
import os
import time
print('here')
soft_reset()
time.sleep(0.2)

# Config File:
config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value = False)
right_front_reverse = config.get('right_front_reverse', default_value = False)
left_rear_reverse = config.get('left_rear_reverse', default_value = False)
right_rear_reverse = config.get('right_rear_reverse', default_value = False)    
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 

# Init motors
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=left_front_reverse) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=right_front_reverse) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=left_rear_reverse) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=right_rear_reverse) # motor 4

# left_front_speed = Speed(12)
# right_front_speed = Speed(16)
left_rear_speed = Speed(25)
right_rear_speed = Speed(4)  

# Init Greyscale
gs0 = ADC('A5')
gs1 = ADC('A6')
gs2 = ADC('A7')

# Init Ultrasonic
us = Ultrasonic(Pin('D8'), Pin('D9'))

# Init Servo
# print("Init Servo: %s" % ultrasonic_servo_offset)

servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

def start_speed_thread():
    # left_front_speed.start()
    # right_front_speed.start()
    left_rear_speed.start()
    right_rear_speed.start()

##################################################################
# Grayscale 
def get_grayscale_list():
    adc_value_list = []
    adc_value_list.append(gs0.read())
    adc_value_list.append(gs1.read())
    adc_value_list.append(gs2.read())
    return adc_value_list

def is_on_edge(ref, gs_list):
    ref = int(ref)
    if gs_list[2] <= ref or gs_list[1] <= ref or gs_list[0] <= ref:  
        return True
    else:
        return False

def get_line_status(ref,fl_list):#170<x<300
    ref = int(ref)
    if fl_list[1] <= ref:
        return 0
    
    elif fl_list[0] <= ref:
        return -1

    elif fl_list[2] <= ref:
        return 1

########################################################
# Ultrasonic
ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []

errors = []

def run_command(cmd=""):
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    # print(result)
    # print(status)
    return status, result


def do(msg="", cmd=""):
    print(" - %s..." % (msg), end='\r')
    print(" - %s... " % (msg), end='')
    status, result = eval(cmd)
    # print(status, result)
    if status == 0 or status == None or result == "":
        print('Done')
    else:
        print('Error')
        errors.append("%s error:\n  Status:%s\n  Error:%s" %
                      (msg, status, result))

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    return distance

def get_status_at(angle, ref1=35, ref2=10):
    dist = get_distance_at(angle)
    if dist > ref1 or dist == -2:
        return 2
    elif dist > ref2:
        return 1
    else:
        return 0

def scan_step(ref):
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = get_status_at(current_angle, ref1=ref)#ref1

    scan_list.append(status)
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False

########################################################
# Motors
def forward(power):
    with left_front.lock():
        with left_rear.lock():
            with right_front.lock():
                with right_rear.lock():
                    left_front.set_power(power)
                    left_rear.set_power(power)
                    right_front.set_power(power)
                    right_rear.set_power(power)

def backward(power):
    with left_front.lock():
        with left_rear.lock():
            with right_front.lock():
                with right_rear.lock():
                    left_front.set_power(-power)
                    left_rear.set_power(-power)
                    right_front.set_power(-power)
                    right_rear.set_power(-power)

def turn_left(power):
    with left_front.lock():
        with left_rear.lock():
            with right_front.lock():
                with right_rear.lock():
                    left_front.set_power(-power)
                    left_rear.set_power(-power)
                    right_front.set_power(power)
                    right_rear.set_power(power)

def turn_right(power):
    with left_front.lock():
        with left_rear.lock():
            with right_front.lock():
                with right_rear.lock():
                    left_front.set_power(power)
                    left_rear.set_power(power)
                    right_front.set_power(-power)
                    right_rear.set_power(-power)

def stop():
    with left_front.lock():
        with left_rear.lock():
            with right_front.lock():
                with right_rear.lock():
                    left_front.set_power(0)
                    left_rear.set_power(0)
                    right_front.set_power(0)
                    right_rear.set_power(0)

def set_motor_power(motor, power):
    if motor == 1:
        left_front.set_power(power)
    elif motor == 2:
        right_front.set_power(power)
    elif motor == 3:
        left_rear.set_power(power)
    elif motor == 4:
        right_rear.set_power(power)

# def speed_val(*arg):
#     if len(arg) == 0:
#         return (left_front_speed() + left_rear_speed() + right_front_speed() + right_rear_speed()) / 4
#     elif arg[0] == 1:
#         return left_front_speed()
#     elif arg[0] == 2:
#         return right_front_speed()
#     elif arg[0] == 3:
#         return left_rear_speed()
#     elif arg[0] == 4:
#         return right_rear_speed()

def speed_val():
    return (left_rear_speed() + right_rear_speed()) / 2.0

######################################################## 
if __name__ == '__main__':
    start_speed_thread()
    while 1:
        forward(1)
        time.sleep(0.1)
        print(speed_val())
#!/usr/bin/env python3

class ADC(I2C):
    ADDR=0x14                   # i2c_address 0x14

    def __init__(self, chn):    # adc channel:"A0, A1, A2, A3, A4, A5, A6, A7"
        super().__init__()
        if isinstance(chn, str):
            if chn.startswith("A"):     
                chn = int(chn[1:])
            else:
                raise ValueError("ADC channel should be between [A0, A7], not {0}".format(chn))
        if chn < 0 or chn > 7:          
            self._error('Incorrect channel range')
        chn = 7 - chn
        self.chn = chn | 0x10          
        self.reg = 0x40 + self.chn
        # self.bus = smbus.SMBus(1)
        
    def read(self):                     
        self.send([self.chn, 0, 0], self.ADDR)

        # self._debug("Read from 0x%02X"%(self.ADDR))
        # value_h = self.bus.read_byte(self.ADDR)
        value_h = self.recv(1, self.ADDR)[0]            

        # self._debug("Read from 0x%02X"%(self.ADDR))
        # value_l = self.bus.read_byte(self.ADDR)
        value_l = self.recv(1, self.ADDR)[0]           

        value = (value_h << 8) + value_l
        # self._debug("Read value: %s"%value)
        return value


def test():
    import time
    adc = ADC(0)
    while True:
        print(adc.read())
        time.sleep(1)

if __name__ == '__main__':
    test()

class FileDB(object):
	"""A file based database.

    A file based database, read and write arguements in the specific file.
    """
	import os

	# user_name = os.getlogin()
	# user_name = os.popen("echo ${SUDO_USER:-$(who -m | awk '{ print $1 }')}").readline().strip()
	# user_name = os.popen("getent passwd ${SUDO_UID:-$(id -u)} | cut -d: -f 6").readline().strip().split('/')[2]
	user_name = os.popen("ls /home | head -n 1").readline().strip()



	DIR = f"/home/{user_name}/.picar-4wd/"
	def __init__(self, db=None):
		'''Init the db_file is a file to save the datas.'''

		# Check if db_file is defined
		if db != None:
			self.db = db
		else:
			self.db = "config"

	def get(self, name, default_value=None):
		"""Get value by data's name. Default value is for the arguemants do not exist"""
		try:
			conf = open(self.DIR+self.db,'r')
			lines=conf.readlines()
			conf.close()
			flag = False
			# Find the arguement and set the value
			for line in lines:
				# print(line)
				if line.startswith('#'):
					continue
				# print("no#")
				if line.split('=')[0].strip() == name:
					# print(name)
					value = line.split('=')[1].replace(' ', '').strip()
					break
			else:
				# print("flag_error")
				return default_value

			return eval(value)

		except Exception as e:
			print('error: %s'%e)
			return default_value
	
	def set(self, name, value):
		"""Set value by data's name. Or create one if the arguement does not exist"""

		# Read the file
		conf = open(self.DIR+self.db,'r')
		lines=conf.readlines()
		conf.close()
		flag = False
		# Find the arguement and set the value
		for i, line in enumerate(lines):
			if line.startswith('#'):
				continue
			if line.split('=')[0].strip() == name:
				lines[i] = '%s = %s\n' % (name, value)
				break
		# If arguement does not exist, create one
		else:
			lines.append('%s = %s\n\n' % (name, value))

		# Save the file
		conf = open(self.DIR+self.db,'w')
		conf.writelines(lines)
		conf.close()

def test():
	name = "hhh"
	db = FileDB()
	print("Get not exist: %s" % db.get(name, 0))
	print("Set not exist: %s" % db.set(name, 10))
	print("Get exist: %s" % db.get(name, 0))
	print("Set exist: %s" % db.set(name, 20))
	print("Get exist: %s" % db.get(name, 0))

if __name__ == "__main__":
	test()
     
from smbus2 import SMBus
from .utils import soft_reset
import time

class I2C(object):
    MASTER = 0
    SLAVE  = 1
    RETRY = 5

    def __init__(self, *args, **kargs):    
        self._bus = 1
        self._smbus = SMBus(self._bus)

    def auto_reset(func):
        def wrapper(*args, **kw):
            try:
                return func(*args, **kw)
            except OSError:
                soft_reset()
                time.sleep(1)
                return func(*args, **kw)
        return wrapper

    @auto_reset
    def _i2c_write_byte(self, addr, data):   
        # self._debug("_i2c_write_byte: [0x{:02X}] [0x{:02X}]".format(addr, data))
        return self._smbus.write_byte(addr, data)
    
    @auto_reset
    def _i2c_write_byte_data(self, addr, reg, data):
        # self._debug("_i2c_write_byte_data: [0x{:02X}] [0x{:02X}] [0x{:02X}]".format(addr, reg, data))
        return self._smbus.write_byte_data(addr, reg, data)
    
    @auto_reset
    def _i2c_write_word_data(self, addr, reg, data):
        # self._debug("_i2c_write_word_data: [0x{:02X}] [0x{:02X}] [0x{:04X}]".format(addr, reg, data))
        return self._smbus.write_word_data(addr, reg, data)
    
    @auto_reset
    def _i2c_write_i2c_block_data(self, addr, reg, data):
        # self._debug("_i2c_write_i2c_block_data: [0x{:02X}] [0x{:02X}] {}".format(addr, reg, data))
        return self._smbus.write_i2c_block_data(addr, reg, data)
    
    @auto_reset
    def _i2c_read_byte(self, addr):  
        # self._debug("_i2c_read_byte: [0x{:02X}]".format(addr))
        return self._smbus.read_byte(addr)

    @auto_reset
    def _i2c_read_i2c_block_data(self, addr, reg, num):
        # self._debug("_i2c_read_i2c_block_data: [0x{:02X}] [0x{:02X}] [{}]".format(addr, reg, num))
        return self._smbus.read_i2c_block_data(addr, reg, num)

    def is_ready(self, addr):
        addresses = self.scan()
        if addr in addresses:
            return True
        else:
            return False

    def scan(self):                            
        cmd = "i2cdetect -y %s" % self._bus
        _, output = self.run_command(cmd)          
        outputs = output.split('\n')[1:]       
       # self._debug("outputs")
        addresses = []
        for tmp_addresses in outputs:
            tmp_addresses = tmp_addresses.split(':')[1]
            tmp_addresses = tmp_addresses.strip().split(' ')    
            for address in tmp_addresses:
                if address != '--':
                    addresses.append(address)
     #   self._debug("Conneceted i2c device: %s"%addresses)                   
        return addresses

    def send(self, send, addr, timeout=0):                     
        if isinstance(send, bytearray):
            data_all = list(send)
        elif isinstance(send, int):
            data_all = []
            d = "{:X}".format(send)
            d = "{}{}".format("0" if len(d)%2 == 1 else "", d)  
            # print(d)
            for i in range(len(d)-2, -1, -2):      
                tmp = int(d[i:i+2], 16)             
                # print(tmp)
                data_all.append(tmp)                
            data_all.reverse()
        elif isinstance(send, list):
            data_all = send
        else:
            raise ValueError("send data must be int, list, or bytearray, not {}".format(type(send)))

        if len(data_all) == 1:                      
            data = data_all[0]
            self._i2c_write_byte(addr, data)
        elif len(data_all) == 2:                    
            reg = data_all[0]
            data = data_all[1]
            self._i2c_write_byte_data(addr, reg, data)
        elif len(data_all) == 3:                    
            reg = data_all[0]
            data = (data_all[2] << 8) + data_all[1]
            self._i2c_write_word_data(addr, reg, data)
        else:
            reg = data_all[0]
            data = list(data_all[1:])
            self._i2c_write_i2c_block_data(addr, reg, data)

    def recv(self, recv, addr=0x00, timeout=0):     
        if isinstance(recv, int):                   
            result = bytearray(recv)
        elif isinstance(recv, bytearray):
            result = recv
        else:
            return False
        for i in range(len(result)):
            result[i] = self._i2c_read_byte(addr)
        return result

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8): #memaddr match to chn
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, int):
            data_all = []
            for i in range(0, 100):
                d = data >> (8*i) & 0xFF
                if d == 0:
                    break
                else:
                    data_all.append(d)
            data_all.reverse()
        self._i2c_write_i2c_block_data(addr, memaddr, data_all)
    
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):     
        if isinstance(data, int):
            num = data
        elif isinstance(data, bytearray):
            num = len(data)
        else:
            return False
        result = bytearray(num)
        result = self._i2c_read_i2c_block_data(addr, memaddr, num)
        return result

    def test():
        a_list = [0x2d,0x64,0x0]
        b = I2C()
        b.send(a_list,0x14)
class Motor():
    STEP = 10
    DELAY = 0.1
    LOCK = threading.Lock()
    def __init__(self, pwm_pin, dir_pin, is_reversed=False):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self._is_reversed = is_reversed
        self._power = 0
        self._except_power = 0
    
    # def start_timer(self):
    #     self.t = threading.Timer(self.DELAY, self.adder_thread)
    #     self.t.start()

    def set_power(self, power):
        with self.lock:
            if power >= 0:
                direction = 0
            elif power < 0:
                direction = 1
            power = abs(power)
            if power != 0:
                power = int(power /2 ) + 50
            power = power

            direction = direction if not self._is_reversed else not direction  
            self.dir_pin.value(direction)
                
            self.pwm_pin.pulse_width_percent(power)

#     def adder_thread(self):
#         if self._except_power > self._power:
#             step = self.STEP
#         else:
#             step = -self.STEP
#         if abs(self._except_power - self._power) < self.STEP:
#             self._power = self._except_power
#         else:
#             self._power += step
#         self._set_power(self._power)
#         if self._power != self._except_power:
#             self.start_timer()

#     def set_power(self, power):
#         # print("Power: {}".format(power))
#         self._except_power = power
#         if self._power != self._except_power:
#             self.start_timer()

# if __name__ == "__main__":
#     import picar-4wd as fc
#     import time
#     fc.forward(100)
class Pin(object):
    OUT = GPIO.OUT                  
    IN = GPIO.IN                   
    IRQ_FALLING = GPIO.FALLING      
    IRQ_RISING = GPIO.RISING        
    IRQ_RISING_FALLING = GPIO.BOTH  
    PULL_UP = GPIO.PUD_UP           
    PULL_DOWN = GPIO.PUD_DOWN       
    PULL_NONE = None                
    _dict = {                       
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
    }

    def __init__(self, *value):
        super().__init__()          
        GPIO.setmode(GPIO.BCM)      
        GPIO.setwarnings(False)     
        if len(value) > 0:          
            pin = value[0]
        if len(value) > 1:          
            mode = value[1]
        else:
            mode = None
        if len(value) > 2:          
            setup = value[2]
        else:
            setup = None
        if isinstance(pin, str):    
            try:
                self._bname = pin
                self._pin = self.dict()[pin]
            except Exception as e:
                print(e)
                self._error('Pin should be in %s, not %s' % (self._dict, pin))
        elif isinstance(pin, int):  
            self._pin = pin
        else:
            self._error('Pin should be in %s, not %s' % (self._dict, pin))
        self._value = 0
        self.init(mode, pull=setup)
    #    self._info("Pin init finished.")
        
    def init(self, mode, pull=PULL_NONE):   
        self._pull = pull
        self._mode = mode
        if mode != None:
            if pull != None:
                GPIO.setup(self._pin, mode, pull_up_down=pull)
            else:
                GPIO.setup(self._pin, mode)

    def dict(self, *_dict):                 
        if len(_dict) == 0:                 
            return self._dict
        else:
            if isinstance(_dict, dict):
                self._dict = _dict
            else:
                self._error(
                    'argument should be a pin dictionary like {"my pin": ezblock.Pin.cpu.GPIO17}, not %s' % _dict)

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):                 
        if len(value) == 0:
            self.mode(self.IN)
            result = GPIO.input(self._pin)
        #    self._debug("read pin %s: %s" % (self._pin, result))
            return result
        else:                               
            value = value[0]
            self.mode(self.OUT)
            GPIO.output(self._pin, value)
            return value

    def on(self):                           
        return self.value(1)

    def off(self):                          
        return self.value(0)

    def high(self):                        
        return self.on()

    def low(self):                          
        return self.off()

    def mode(self, *value):                 
        if len(value) == 0:
            return self._mode
        else:
            mode = value[0]
            self._mode = mode
            GPIO.setup(self._pin, mode)

    def pull(self, *value):     
        return self._pull

    def irq(self, handler=None, trigger=None):      
        self.mode(self.IN)
        GPIO.add_event_detect(self._pin, trigger, callback=handler)

    def name(self):                                 
        return "GPIO%s"%self._pin

    def names(self):
        return [self.name, self._bname]

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4  = 4
        GPIO5  = 5
        GPIO6  = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass
class PWM(I2C):
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44
    ADDR = 0x14
    CLOCK = 72000000

    def __init__(self, channel):
        super().__init__()
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError("PWM channel should be between [P1, P14], not {0}".format(channel))
        try:
            self.send(0x2C, self.ADDR)
            self.send(0, self.ADDR)
            self.send(0, self.ADDR)
        except IOError:
            self.ADDR = 0x15

      #  self.debug = debug
      #  self._debug("PWM address: {:02X}".format(self.ADDR))
        self.channel = channel
        self.timer = int(channel/4)
        self.bus = smbus.SMBus(1)
        self._pulse_width = 0
        self._freq = 50
        self.freq(50)

    def i2c_write(self, reg, value):
        value_h = value >> 8
        value_l = value & 0xff
    #   self._debug("i2c write: [0x%02X, 0x%02X, 0x%02X, 0x%02X]"%(self.ADDR, reg, value_h, value_l))
        self.send([reg, value_h, value_l], self.ADDR)

    def freq(self, *freq):
        if len(freq) == 0:
            return self._freq
        else:
            self._freq = int(freq[0])
            # [prescaler,arr] list
            result_ap = []
            # accuracy list
            result_acy = []
            # middle value for equal arr prescaler
            st = int(math.sqrt(self.CLOCK/self._freq))
            # get -5 value as start
            st -= 5
            # prevent negetive value
            if st <= 0:
                st = 1
            for psc in range(st,st+10):
                arr = int(self.CLOCK/self._freq/psc)
                result_ap.append([psc, arr])
                result_acy.append(abs(self._freq-self.CLOCK/psc/arr))
            i = result_acy.index(min(result_acy))
            psc = result_ap[i][0]
            arr = result_ap[i][1]
        #   self._debug("prescaler: %s, period: %s"%(psc, arr))
            self.prescaler(psc)
            self.period(arr)

    def prescaler(self, *prescaler):
        if len(prescaler) == 0:
            return self._prescaler
        else:
            self._prescaler = int(prescaler[0]) - 1
            reg = self.REG_PSC + self.timer
        #    self._debug("Set prescaler to: %s"%self._prescaler)
            self.i2c_write(reg, self._prescaler)

    def period(self, *arr):
        if len(arr) == 0:
            return self._arr
        else:
            self._arr = int(arr[0]) - 1
            reg = self.REG_ARR + self.timer
        #    self._debug("Set arr to: %s"%self._arr)
            self.i2c_write(reg, self._arr)

    def pulse_width(self, *pulse_width):
        if len(pulse_width) == 0:
            return self._pulse_width
        else:
            self._pulse_width = int(pulse_width[0])
            reg = self.REG_CHN + self.channel
            # CCR = int(self._pulse_width/self.PRECISION * self._arr)
            # print("CCR: %s"%CCR)
            self.i2c_write(reg, self._pulse_width)

    def pulse_width_percent(self, *pulse_width_percent):
        if len(pulse_width_percent) == 0:
            return self._pulse_width_percent
        else:
            self._pulse_width_percent = pulse_width_percent[0] / 100.0
            pulse_width = self._pulse_width_percent * self._arr
            self.pulse_width(pulse_width)

        
def test():
    import time
    p = PWM('P12')
    # p.debug = 'debug'
    p.period(1000)
    p.prescaler(10)
    # p.pulse_width(2048)
    while True:
        for i in range(0, 4095, 10):
            p.pulse_width(i)
            print(i)
            time.sleep(1/4095)
        time.sleep(1)
        for i in range(4095, 0, -10):
            p.pulse_width(i)
            print(i)
            time.sleep(1/4095)
        time.sleep(1)

if __name__ == '__main__':
    test()

from .utils import mapping

class Servo():
    PERIOD = 4095
    PRESCALER = 10
    MAX_PW = 2500
    MIN_PW = 500
    FREQ = 50
    ARR = 4095
    CPU_CLOCK = 72000000
    def __init__(self, pin, offset=0):
        self.pin = pin
        self.offset = offset
        self.pin.period(self.PERIOD)
        prescaler = int(float(self.CPU_CLOCK) / self.FREQ/ self.ARR)
        self.pin.prescaler(prescaler)

    def set_angle(self, angle):
        try:
            angle = int(angle)
        except:
            raise ValueError("Angle value should be int value, not %s"%angle)
        if angle < -90:
            angle = -90
        if angle > 90:
            angle = 90
        angle = angle + self.offset
        High_level_time = mapping(angle, -90, 90, self.MIN_PW, self.MAX_PW)
        pwr =  High_level_time / 20000
        value = int(pwr*self.PERIOD)
        self.pin.pulse_width(value)
class Ultrasonic():
    ANGLE_RANGE = 180
    STEP = 18

    def __init__(self, trig, echo, timeout=0.01):
        self.timeout = timeout
        self.trig = trig
        self.echo = echo
        # Init Servo
        self.servo = Servo(PWM("P0"), offset=10)
        self.angle_distance = [0,0]
        self.current_angle = 0
        self.max_angle = self.ANGLE_RANGE/2
        self.min_angle = -self.ANGLE_RANGE/2
        self.scan_list = []

    def get_distance(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm

    # def get_distance_at(self, angle):
    #     self.servo.set_angle(angle)
    #     time.sleep(0.04)
    #     distance = self.get_distance()
    #     self.angle_distance = [angle, distance]
    #     return distance

    # def get_status_at(self, angle, ref1=35, ref2=10):
    #     dist = self.get_distance_at(angle)
    #     if dist > ref1 or dist == -2:
    #         return 2
    #     elif dist > ref2:
    #         return 1
    #     else:
    #         return 0

    # def scan_step(self, ref):
    #     if self.current_angle >= self.max_angle:
    #         self.current_angle = self.max_angle
    #         us_step = -self.STEP
    #     elif self.current_angle <= self.min_angle:
    #         self.current_angle = self.min_angle
    #         us_step = self.STEP
    #     self.current_angle += us_step
    #     status = self.get_status_at(self.current_angle, ref1=ref)#ref1避障基准值，ref2跟随小车后退时基准值

    #     self.scan_list.append(status)
    #     if self.current_angle == self.min_angle or self.current_angle == self.max_angle:
    #         if us_step < 0:
    #             # print("reverse")
    #             self.scan_list.reverse()
    #         # print(self.scan_list)
    #         self.scan_list = []
    #         return self.scan_list
    #     else:
    #         return False


# user_name = os.getlogin()
# user_name = os.popen("echo ${SUDO_USER:-$(who -m | awk '{ print $1 }')}").readline().strip()
# user_name = os.popen("getent passwd ${SUDO_UID:-$(id -u)} | cut -d: -f 6").readline().strip().split('/')[2]
user_name = os.popen("ls /home | head -n 1").readline().strip()


def soft_reset():
    from .pin import Pin
    soft_reset_pin = Pin("D16")
    soft_reset_pin.low()
    time.sleep(0.01)
    soft_reset_pin.high()
    time.sleep(0.01)

def mapping(x,min_val,max_val,aim_min,aim_max):
    x = aim_min + abs((x - min_val) / (max_val- min_val) * (aim_max-aim_min))
    return x

def cpu_temperature():          # cpu_temperature
    raw_cpu_temperature = subprocess.getoutput("cat /sys/class/thermal/thermal_zone0/temp")
    cpu_temperature = round(float(raw_cpu_temperature)/1000,2)               # convert unit
    #cpu_temperature = 'Cpu temperature : ' + str(cpu_temperature)
    return cpu_temperature

def gpu_temperature():          # gpu_temperature(
    raw_gpu_temperature = subprocess.getoutput( 'vcgencmd measure_temp' )
    gpu_temperature = round(float(raw_gpu_temperature.replace( 'temp=', '' ).replace( '\'C', '' )), 2)
    #gpu_temperature = 'Gpu temperature : ' + str(gpu_temperature)
    return gpu_temperature

def cpu_usage():                # cpu_usage
    # result = str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print($2)}'").readline().strip())
    result = os.popen("mpstat").read().strip()
    result = result.split('\n')[-1].split(' ')[-1]
    result = round(100 - float(result), 2)
    result = str(result)
    # print(result)
    return result

def disk_space():               # disk_space
    p = os.popen("df -h /")
    i = 0
    while 1:
        i = i +1
        line = p.readline()         
        if i==2:
            return line.split()[1:5]    

def ram_info():
    p = os.popen('free')
    i = 0
    while 1:
        i = i + 1
        line = p.readline()
        if i==2:
            return list(map(lambda x:round(int(x) / 1000,1), line.split()[1:4]))   

def pi_read():
    result = {
        "cpu_temperature": cpu_temperature(), 
        "gpu_temperature": gpu_temperature(),
        "cpu_usage": cpu_usage(), 
        "disk": disk_space(), 
        "ram": ram_info(), 
        "battery": power_read(), 
    }
    return result 

def power_read():
    from picar_4wd.adc import ADC
    power_read_pin = ADC('A4')
    power_val = power_read_pin.read()
    power_val = power_val / 4095.0 * 3.3
    # print(power_val)
    power_val = power_val * 3
    power_val = round(power_val, 2)
    return power_val

def getIPs(ifaces=['wlan0', 'eth0']):
    import re
    if isinstance(ifaces, str):
        ifaces = [ifaces]
    ips = []
    for iface in list(ifaces):
        search_str = 'ip addr show {}'.format(iface)
        result = os.popen(search_str).read()
        com = re.compile(r'(?<=inet )(.*)(?=\/)', re.M)
        ipv4 = re.search(com, result)
        if ipv4:
            ip = ipv4.groups()[0]
            ips.append(ip)
    return ips

def getIP(ifaces=['wlan0', 'eth0']):
    ips = getIPs(ifaces)
    if ips:
        return ips[0]
    return None

def main():
    import sys
    if len(sys.argv) >= 2:
        print("Welcome to SunFounder PiCar-4WD.")
        command = sys.argv[1]
        if command == "soft-reset":
            print("soft-reset")
            soft_reset()
        elif command == "power-read":
            print("power-read")
            print("Power voltage: {}V".format(power_read()))
        elif command == "web-example":
            if len(sys.argv) >= 3:
                opt = sys.argv[2]
                if opt == "enable":
                    os.system("sudo update-rc.d picar-4wd-web-example defaults")
                    print("web-example start on boot is enabled")
                elif opt == "disable":
                    os.system("sudo update-rc.d picar-4wd-web-example remove")
                    print("web-example start on boot is disabled")
                else:
                    usage(command)
            else:
                print("Run: `picar-4wd web-example enable/disable` to enable/disable start on boot")
                os.system(f"sudo python3 /home/{user_name}/picar-4wd/examples/web/start.py")
        elif command == "test":
            from picar_4wd import forward, get_distance_at, get_grayscale_list,stop
            if len(sys.argv) >= 3:
                opt = sys.argv[2]
                if opt == "motor":
                    print("Motor test start!, Ctrl+C to Stop")
                    forward(50)
                    try:
                        while True:
                            pass
                    except KeyboardInterrupt:
                        pass
                    finally:
                        stop()
                        time.sleep(0.1)
                elif opt == "servo":
                    print(get_distance_at(0))
                elif opt == "grayscale":
                    print(get_grayscale_list())
                else:
                    usage(command)
        else:
            print('Command error, "%s" is not in list' % sys.argv[1])
            usage()
    else:
        usage()
    destroy()

# def main():
#     try:
#         _main()
#     finally:

def destroy():
    quit()
 
def usage(cmd=None):
    general = '''
Usage:  picar-4wd [Command] [option]

Commands:
    soft-reset
    power-read
    web-example
    test
'''
    web_example = '''
Usage: picar-4wd web-example [option]

Options:
    enable    Enable start on boot
    disable   Disable start on boot
'''
    test = '''
Usage: picar-4wd test [option]

Options:
    motor      test the motor
    servo      test the servo
    grayscale  test the grayscale

'''
    if cmd == None:
        print(general)
    elif cmd == "web-example":
        print(web_example)
    elif cmd == "test":
        print(test)
    destroy()
        
