# ROS Bittleet

This package allows control over [Petoi's Bittle](https://www.petoi.com/bittle) via Raspberry Pi. 

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

## Build 

This package should be cloned into your catkin workspace.

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

