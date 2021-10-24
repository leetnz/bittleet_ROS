# ROS Bittleet

Control [Petoi's Bittle](https://www.petoi.com/bittle) via Raspberry Pi. 

It is a work in progress. 

Useful links:
* [OpenCat Serial Protocol](https://bittle.petoi.com/4-configuration#4-3-arduino-ide-as-an-interface)

## Dependencies

### IO Libraries

This package specifically uses `gpiozero`(python) and `pigpio`(cpp/python)

```sh
sudo apt update
sudo apt install python3-gpiozero
sudo apt install pigpio python3-pigpio
```

The `pigpio` daemon must be running:

```sh
systemctl start pigpiod
```

Optionally:
```sh
systemctl enable pigpiod # Always run
```

### sensor_msgs

We need `sensor_msgs`. Assuming we have installed noetic using https://varhowto.com/install-ros-noetic-raspberry-pi-4/, do the following:
```sh
rosinstall_generator sensor_msgs --rosdistro noetic --deps --wet-only --tar > indigo-sensor_msgs-wet.rosinstall
wstool init src noetic-sensor_msgs-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### GStreamer

GStreamer is used to stream video from Bittle to a ground station running `stream_client.py`. 

Follow the instructions on [hoani.net - GStreamer Setup](https://hoani.net/posts/guides/2021-10-21-gstreamerSetup/)

## Build 

After placing this package in your catkin workspace `src` folder:

```sh
cd $CATKIN_WS
catkin_make
cd -
```

## Run

### XBox Controller

This allows us to teleop bittle using an xbox controller.

To set this up:
* We have a host PC running some derivative of debian.
* The host PC has the `joy` library installed:
    * `sudo apt-get install ros-noetic-joy`
* The host PC has `gstreamer1.0` installed (see [hoani.net - GStreamer Setup](https://hoani.net/posts/guides/2021-10-21-gstreamerSetup/))
* The host PC has `pigpio` installed (see [pigpio download and install](http://abyz.me.uk/rpi/pigpio/download.html))
* This library has been built and sourced on the host PC
* We have an xbox controller plugged into the host PC
* The Bittle RPi is configured with `ROS_HOSTNAME` and `ROS_MASTER_URI` to point to the PC
* Run the following on the PC:
    * `roscore`
    * `rosrun joy joy_node` - publishes xbox controller states
    * `rosrun bittleet stream_client.py` - streams video
* Run the following on the Bittle RPi:
    * `rosrun bittleet xbox_listener`

If all goes well, we should be able to control bittle remotely using the xbox controller.

#### Troubleshooting

* For debugging `joy`, see [ROS - Joy Tutorial](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
* For debugging `gstreamer1.0`:
    * Check if `gst-launch-1.0` works with the commands in the code.
    * Verify your installation following the instructions in [hoani.net - GStreamer Setup](https://hoani.net/posts/guides/2021-10-21-gstreamerSetup/)

### Move Joints Node

```sh
rosrun bittleet move
# or
rosrun bittleet move.py
```

To move a joint, you can run the `/bittleet_move/move` service:

```sh
rosservice call /bittleet_move/move "{index: 8, angle: 45}" 
```
The result will be the returned ascii bytes from Bittle.

