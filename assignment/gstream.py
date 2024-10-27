# save as rtsp_server.py
from gi.repository import Gst, GstRtspServer, GObject, GLib

class RTSPServer(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super(RTSPServer, self).__init__()
        self.set_launch("( v4l2src device=/dev/video0 ! video/x-raw,format=I420,width=320,height=240,framerate=15/1 ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay config-interval=1 pt=96 )")


# GObject.threads_init()
Gst.init(None)

server = GstRtspServer.RTSPServer()
server.set_service("8554")
factory = RTSPServer()
factory.set_shared(True)
server.get_mount_points().add_factory("/stream", factory)
server.attach(None)

loop = GLib.MainLoop()
loop.run()
