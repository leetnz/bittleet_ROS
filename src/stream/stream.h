//
// Stream - GStreamer Wrapper
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#ifndef _BITTLEET_ROS_STREAM_STREAM_H_
#define _BITTLEET_ROS_STREAM_STREAM_H_

extern "C"
{
#include <gst/gst.h>
}

#include <string>

class Stream
{
public:
    Stream(const std::string &stream);
    void shutdown();
    bool isRunning();
    static void mainInit(int argc, char *argv[]);

protected:
    static bool _initOnce;

    GstElement *_pipeline;
    GstBus *_bus;
};

#endif // _BITTLEET_ROS_STREAM_STREAM_H_
