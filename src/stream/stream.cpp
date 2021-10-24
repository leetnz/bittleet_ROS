//
// Stream - GStreamer Wrapper
//
// Hoani Bryson (github.com/hoani)
// Copyright (c) 2021 Leetware Limited.
// License - MIT
//

#include "stream.h"

#include <iostream>


Stream::Stream(const std::string &stream)
{
    if (!_initOnce)
    {
        std::cout << "Call Stream::mainInit(argc, argv) before creating a stream object!" << std::endl;
    }

    _pipeline = gst_parse_launch(stream.c_str(), NULL);
    gst_element_set_state(_pipeline, GST_STATE_PLAYING);
    _bus = gst_element_get_bus(_pipeline);
}

void Stream::shutdown()
{
    gst_object_unref(_bus);
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
}

bool Stream::isRunning() {
    GstMessage* msg = gst_bus_pop_filtered(_bus, (GstMessageType)((int)GST_MESSAGE_ERROR | (int)GST_MESSAGE_EOS));
    if (msg != NULL) {
        gst_message_unref(msg);
        return false;
    }
    return true;
}

void Stream::mainInit(int argc, char *argv[])
{
    if (!_initOnce)
    {
        /* Initialize GStreamer */
        gst_init(&argc, &argv);
        _initOnce = true;
    }
}

bool Stream::_initOnce = false;
