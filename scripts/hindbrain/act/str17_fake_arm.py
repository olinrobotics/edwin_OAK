#!/usr/bin/env python

"""
This code intializes all the methods of the str17_serial_control script but does nothing when called
Used for debug purposes
"""

import rospy

class StArm():
    def __init__(self):
        rospy.init_node('robot_fake_arm', anonymous=True)
        self.pub = rospy.Publisher('arm_debug', String, queue_size=10)

    def initial_calibration(self):
        pass

    def purge(self):
        pass

    def roboforth(self):
        pass

    def decimal(self):
        pass

    def start(self):
        pass

    def execute_command(self, cmd):
        pass

    def continuous(self):
        pass

    def segmented(self):
        pass

    def joint(self):
        pass

    def create_route(self, route_name, commands, debug = False):
        pass

    def calibrate(self):
        pass

    def home(self):
        pass

    def cartesian(self, block=False):
        pass

    def block_on_result(self, cmd, debug=False):
        pass

    def get_status(self):
        pass

    def get_speed(self):
        pass

    def set_speed(self, speed):
        pass

    def set_point(self, name):
        pass

    def get_accel(self):
        pass

    def set_accel(self, accel):
        pass

    def run_route(self, route):
        pass

    def move_to(self, x, y, z, debug=False, block=True):
        pass

    def rotate_wrist(self, roll):
        pass

    def rotate_wrist_rel(self, roll_inc):
        pass

    def rotate_hand(self, pitch):
        pass

    def rotate_elbow(self, pitch):
        pass

    def rotate_shoulder(self, pitch):
        pass

    def rotate_waist(self, pitch):
        pass

    def rotate_waist_rel(self, pitch):
        pass

    def rotate_hand_rel(self, pitch_inc):
        pass

    def move_hand(self, roll):
        pass

    def energize(self):
        pass

    def de_energize(self):
        pass

    def where(self):
        pass

    def check_if_done(self):
        pass

    def dummy(self):
        pass

    def lock_wrist_angle(self,TF = True):
        pass
