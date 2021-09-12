#!/usr/bin/env python

import pigpio
import time

import rospy
from bittleet.srv import MoveServo, MoveServoRequest, MoveServoResponse

class Move:
    JOINT_INDEXES = [0, 8, 9, 10, 11, 12, 13, 14, 15]

    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            return
        self.handle = self.pi.serial_open("/dev/ttyS0", 115200)

        self.node = rospy.init_node('bittleet_move')

    def __del__(self):
        if self.handle != None:
            self.pi.serial_close(self.handle)

    def run(self):
        if not self.pi.connected:
            return
        self.srv = rospy.Service(rospy.get_name() +'/move', MoveServo, self.handle_move)
        rospy.spin()

    def handle_move(self, req: MoveServoRequest):
        angle = req.angle
        if not -90.0 <= angle <= 90.0:
            raise rospy.ServiceException("angle {} out of range".format(angle))
        index = req.index
        if index not in Move.JOINT_INDEXES:
            raise rospy.ServiceException("index {} invalid".format(index))

        done = False
        res = MoveServoResponse()
        command = f"m {index} {angle:.0f} "
        print(f"Sending '{command}'")
        
        while not done:
            self.pi.serial_write(self.handle, command)

            time.sleep(0.05)
            len, data = self.pi.serial_read(self.handle, 100)

            if len > 0:
                try:
                    res.result = data.decode("utf-8")
                    done = True
                except:
                    pass
            else:
                res.result = "<No Reply>"
                done = True


        return res


if __name__ == "__main__":
    try:
        move = Move()
        move.run()
    except rospy.ROSInterruptException:
        pass

