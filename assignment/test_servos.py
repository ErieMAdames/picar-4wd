import subprocess
import socket
import json
from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM
import os

# Setup your servos and initial angles
servo0_angle_offset = 0
servo1_angle_offset = -8
servo2_angle_offset = 0
servo0_angle = 0
servo1_angle = 0
servo2_angle = 0
servo0 = Servo(PWM("P0"), servo0_angle_offset)
servo1 = Servo(PWM("P1"), servo1_angle_offset)
servo2 = Servo(PWM("P2"), servo2_angle_offset)

servo0.set_angle(servo0_angle_offset)
servo1.set_angle(servo1_angle_offset)
servo2.set_angle(servo2_angle_offset)

# Set ulimit for file descriptors
os.system('ulimit -n 4096')

# # Start RTSP stream using ffmpeg
# ffmpeg_command = [
#     'ffmpeg', '-f', 'v4l2', '-i', '/dev/video0', '-preset', 'ultrafast',
#     '-f', 'rtsp', 'rtsp://192.168.86.46:8554/stream'
# ]
# ffmpeg_process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

HOST = "192.168.86.46" # IP address of your Raspberry PI
PORT = 65432
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
                    servo2_angle = max(servo2_angle - 2, -90)
                    servo2.set_angle(servo2_angle)
                elif (data == b"down"):
                    servo2_angle = min(servo2_angle + 2, 90)
                    servo2.set_angle(servo2_angle)
                elif (data == b"left"):
                    servo1_angle = max(servo1_angle - 2, -90)
                    servo1.set_angle(servo1_angle)
                elif (data == b"right"):
                    servo1_angle = min(servo1_angle + 2, 90)
                    servo1.set_angle(servo1_angle)
                client.sendall(str.encode(json.dumps({
                    'data': data.decode("utf-8"),
                    'servo1_angle': servo1_angle,
                    'servo2_angle': servo2_angle
                }))) # Echo back to client
    except (KeyboardInterrupt, Exception) as e:
        print(e)
        print("Closing socket and terminating stream")
        # ffmpeg_process.terminate()  # Stop ffmpeg when done
        # ffmpeg_process.wait()  # Wait for ffmpeg to fully close
        client.close()
        s.close()
socket.socket.close()