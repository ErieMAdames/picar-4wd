from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM
import socket
import json
import time


servo0 = Servo(PWM("P0"))
servo1 = Servo(PWM("P1"))
servo2 = Servo(PWM("P2"))

servo0.set_angle(0)
servo1.set_angle(0)
servo2.set_angle(0)

import os

os.system('ulimit -n 4096') 

HOST = "192.168.86.46" # IP address of your Raspberry PI
PORT = 65432
servo0_angle = 0
servo1_angle = 0
servo2_angle = 0
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    try:
        while 1:
            client, clientInfo = s.accept()
            data = client.recv(1024)
            if data != b"":
                print(data)
                if (data == b"up"):
                    servo1_angle += 1
                    servo1.set_angle(servo1_angle)
                elif (data == b"down"):
                    servo1_angle -= 1
                    servo1.set_angle(servo1_angle)
                elif (data == b"left"):
                    servo2_angle += 1
                    servo2.set_angle(servo2_angle)
                elif (data == b"right"):
                    servo2_angle -= 1
                    servo2.set_angle(servo2_angle)
                client.sendall(str.encode(json.dumps({
                    'data': data.decode("utf-8"),
                    'servo1_angle': servo1_angle,
                    'servo2_angle': servo2_angle
                }))) # Echo back to client
    except (KeyboardInterrupt, Exception) as e:
        print(e)
        print("Closing socket")
        client.close()
        s.close()    