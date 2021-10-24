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

## Host PC Configuration

This package is designed to be used with a host PC which:
* Runs `roscore`
* Recieves streamed video
* Is used for teleoperation

The requirements for the PC are:
* It is some derivate of debian
* We have ros noetic installed

### Host PC Setup

* Install the `joy` library:
    * `sudo apt-get install ros-noetic-joy`
* Install `gstreamer1.0` (see [hoani.net - GStreamer Setup](https://hoani.net/posts/guides/2021-10-21-gstreamerSetup/))
* Download and install `pigpio` from source (see [pigpio download and install](http://abyz.me.uk/rpi/pigpio/download.html))
* Download this package, build it and source it.
* I have found that I have needed to add the raspberry pi config to `/etc/hosts`, usually something like:
    * `192.168.1.123	rpi-bittle`
    * This tends to smooth over the name resolution so that the raspberry pi can publish topics correctly

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
* We have an xbox controller plugged into the host PC
* The Bittle RPi is configured with `ROS_MASTER_URI` to point to the PC uri
    * Usually something like `export ROS_MASTER_URI=http://192.168.1.10:11311` in `.bashrc`
* Run the following on the PC:
    * `roscore`
    * `rosrun joy joy_node` - publishes xbox controller states
    * `rosrun bittleet stream_client.py` - streams video
* Run the following on the Bittle RPi:
    * `rosrun bittleet xbox_listener _addr:=$(hostname)`

If all goes well, we should be able to control bittle remotely using the xbox controller.

#### Controls

| Control | Function |
|:-------:|:---------|
| Left toggle | Movement |
| Left toggle Press | Stand |
| Start | Shutdown servos |
| Left Trigger | Start/Stop Video |
| Right Trigger | Enable/Disable Gyroscope |
| A/B/X/Y       | Run various behaviors |

#### Debug running

To run `xbox_listener` on the host PC without a Raspberry Pi:
```
rosrun bittleet xbox_listener _addr:=$(hostname) _debug:=true _h264:=false
```

If your host PC camera is an `H264` camera, you can set `h264:=true`

#### Troubleshooting

* Debugging `joy`, see [ROS - Joy Tutorial](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
* Debugging `gstreamer1.0`:
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

