# save as rtsp_server.py
from gi.repository import Gst, GstRtspServer, GObject

class RTSPServer(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super(RTSPServer, self).__init__()
        self.set_launch("( v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay name=pay0 pt=96 )")

GObject.threads_init()
Gst.init(None)

server = GstRtspServer.RTSPServer()
factory = RTSPServer()
factory.set_shared(True)
server.get_mount_points().add_factory("/stream", factory)
server.attach(None)

print("RTSP stream available at rtsp://<your-pi-ip>:8554/stream")
loop = GObject.MainLoop()
loop.run()
