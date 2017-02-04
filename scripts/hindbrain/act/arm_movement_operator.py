#!/usr/bin/env python

import rospy
import math
import str17_serial_control
import str17_fake_arm
import numpy as np
from std_msgs.msg import String, Int16
import time

class ArmCommands:
    """
    ArmCommands object details serial level control of the STR17 robotic arm
    """

    def __init__(self, debug = False):
        rospy.init_node('robot_arm', anonymous=True)
        rospy.Subscriber('/arm_cmd', String, self.arm_callback, queue_size=1)

        self.debug = debug
        self.debug_pub = rospy.Publisher('arm_debug', String, queue_size=10)

        if self.debug:
            self.arm = str17_fake_arm.StArm()
        else:
            self.arm = str17_serial_control.StArm()

        self.plan = []

        rospy.loginfo("CALIBRATING")
        self.arm.initial_calibration()
        self.arm.start()

        rospy.loginfo("ARM SPD IS: %s", self.arm.get_speed())
        rospy.loginfo("ARM ACCEL IS: %s", self.arm.get_accel())

        self.arm.home()
        self.debug_pub.publish("HOMING DONE")

    def arm_callback(self, cmdin):
        self.arm.joint()
        cmd = cmdin.data
        cmd = str(cmdin).replace("data: ", "")
        if len(cmd.split(':: ')) > 1:
            param = cmd.split(':: ')[1]
            cmd = cmd.split(':: ')[0]
        rospy.loginfo("Received command: %s", cmd)

        if cmd == "de_energize":
            self.arm.de_energize()
        elif cmd == "energize":
            self.arm.energize()
        elif cmd == "where":
            location = self.arm.where()
            rospy.loginfo(location)
        elif cmd == "create_route":
            param = param.split("; ")
            route_name = param[0]

            commands = []
            numbers = param[1].split(", ")

            if len(numbers)%6 != 0:
                rospy.logwarn("INVALID ROUTE, ignoring command")
                return

            i = 0
            while i < len(numbers):
                route = []
                j = 0
                while j < 6:
                    route.append(int(numbers[i+j]))
                    j += 1
                commands.append(route)
                i += j

            rospy.loginfo("CREATING ROUTE: %s", route_name)
            self.arm.create_route(route_name, commands)

        elif cmd == "calibrate":
            self.arm.calibrate()
        elif cmd == "home":
            self.arm.home()
        elif cmd == "get_speed":
            speed = self.arm.get_speed()
        elif cmd == "set_speed":
            #SPD is also in units of 1000
            self.arm.set_speed(float(param))
            rospy.loginfo("NEW ARM SPD IS: %s", self.arm.get_speed())
        elif cmd == "set_point":
            self.arm.set_point(param)
        elif cmd == "get_accel":
            accel = self.arm.get_accel()
        elif cmd == "set_accel":
            #ACCEL is also in units of 1000
            self.arm.set_accel(float(param))
        elif cmd == "run_route":
            res = self.arm.run_route(param)
            if not res:
                self.debug_pub.publish("ROUTE NOT FOUND: " + param)
        elif cmd == "move_to":
            #NOTE: move_to is in units of mm
            temp = param.split(", ")
            x = temp[0]
            y = temp[1]
            z = temp[2]
            pitch = temp[3]
            self.arm.move_to(x,y,z,self.arm.debug)
        elif cmd == "rotate_wrist":
            self.arm.rotate_wrist(param)
        elif cmd == "rotate_wrist_rel":
            self.arm.rotate_wrist_rel(param)
        elif cmd == "rotate_hand":
            self.arm.rotate_hand(param)
        elif cmd == "rotate_elbow":
            self.arm.rotate_elbow(param)
        elif cmd == "rotate_shoulder":
            self.arm.rotate_shoulder(param)
        elif cmd == "rotate_waist":
            self.arm.rotate_waist(param)
        elif cmd == "rotate_waist_rel":
            self.arm.rotate_waist(param)
        elif cmd == "rotate_hand_rel":
            self.arm.rotate_hand_rel(param)
        elif cmd == "sleeping":
            time.sleep(float(param))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
