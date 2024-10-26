import subprocess
import socket
import json
from picar_4wd.servo import Servo
from picar_4wd.pwm import PWM
import os

# Setup your servos and initial angles (same as before)

# Start RTSP stream using ffmpeg and log output
ffmpeg_command = [
    'ffmpeg', '-f', 'v4l2', '-i', '/dev/video0', '-preset', 'ultrafast',
    '-f', 'rtsp', 'rtsp://192.168.86.46:8554/stream'
]
ffmpeg_process = subprocess.Popen(
    ffmpeg_command,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

# Log ffmpeg output continuously in a separate thread or process
import threading
def log_ffmpeg_output(process):
    for line in process.stderr:
        print("FFmpeg output:", line)

threading.Thread(target=log_ffmpeg_output, args=(ffmpeg_process,)).start()

# Socket server setup (same as before)