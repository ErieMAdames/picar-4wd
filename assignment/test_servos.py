from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM


servo0 = Servo(PWM("P0"))
servo1 = Servo(PWM("P1"))
servo2 = Servo(PWM("P2"))

servo0.set_angle(0)
servo1.set_angle(0)
servo2.set_angle(0)