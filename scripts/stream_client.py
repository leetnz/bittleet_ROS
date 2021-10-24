#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

import sys
import gi

gi.require_version('GLib', '2.0')
gi.require_version('GObject', '2.0')
gi.require_version('Gst', '1.0')

from gi.repository import Gst, GObject, GLib



class Stream:
    initialized = False

    def __init__(self, pipeline):
        self.pipeline = None

        if not Stream.initialized:
            Gst.init(sys.argv[1:])
            Stream.initialized = True

        self.pipeline = Gst.parse_launch(pipeline)
        self.pipeline.set_state(Gst.State.PLAYING)

    def __del__(self):
        self.pipeline.set_state(Gst.State.NULL)

    def isActive(self):
        bus = self.pipeline.get_bus()
        msg = bus.pop_filtered(
            Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
        if msg == None:
            return True

        return False


class Listener:
    def __init__(self):
        self.stream = None

    def run(self):
        rospy.init_node('stream_client', anonymous=True)
        rospy.Subscriber("stream_active", Bool, self.callback)
        rospy.spin()

    def callback(self, streamActive):
        if not self.streamActive():
            if streamActive.data:
                rospy.loginfo("Starting stream")
                # self.stream = Stream("v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink")
                self.stream = Stream("tcpclientsrc host=127.0.0.1 port=5000 ! "
                                "gdpdepay ! "
                                "rtph264depay ! "
                                "avdec_h264 ! "
                                "videoconvert ! "
                                "autovideosink sync=false")
        else:
            if not streamActive.data:
                rospy.loginfo("Closing stream")
                self.closeStream()

    def streamActive(self):
        if self.stream is None:
            return False

        if self.stream.isActive():
            return True
        else:
            self.closeStream()
            return False

    def closeStream(self):
        del self.stream
        self.stream = None   

    

if __name__ == '__main__':
    l = Listener()
    l.run()
