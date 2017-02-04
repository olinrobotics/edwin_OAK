#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String, Int16
import time
from edwin_oak.srv import ArmCommand

class ArmCommands:
    def __init__(self):
        rospy.wait_for_service('arm_cmd')
        self.send_command = rospy.ServiceProxy('arm_cmd', ArmCommand)

    def run(self):
        while True:
            cmd_in = str(raw_input("ROBOFORTH COMMAND: "))
            result = self.send_command("test", "test")
            print result.result

if __name__ == "__main__":
    arm_eng = ArmCommands()
    arm_eng.run()
