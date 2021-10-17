# ROS Bittleet

Control [Petoi's Bittle](https://www.petoi.com/bittle) via Raspberry Pi. 

It is a work in progress. 

Useful links:
* [OpenCat Serial Protocol](https://bittle.petoi.com/4-configuration#4-3-arduino-ide-as-an-interface)

## Dependencies

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

We need `sensor_msgs`. Assuming we have installed noetic using https://varhowto.com/install-ros-noetic-raspberry-pi-4/, do the following:
```sh
rosinstall_generator sensor_msgs --rosdistro noetic --deps --wet-only --tar > indigo-sensor_msgs-wet.rosinstall
wstool init src noetic-sensor_msgs-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Build 

After placing this package in your catkin workspace `src` folder:

```sh
cd $CATKIN_WS
catkin_make
cd -
```

## Run

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

