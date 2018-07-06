#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from wiimote.msg import State
from wiimote.msg import IrSourceInfo
from sensor_msgs.msg import Imu


class PointCalibration:

    def __init__(self):

        self.wii1_detected = False
        self.wii2_detected = False
        self.key_pressed = False

        self.wii_pix1 = Vector3(0., 0., 0.)
        self.wii_pix2 = Vector3(0., 0., 0.)

        # SUBS N PUBS
        self.wii_subscriber1 = rospy.Subscriber(
            "wiimote/state1",
            State,
            self.wii_cb1)

        self.wii_subscriber2 = rospy.Subscriber(
            "wiimote/state2",
            State,
            self.wii_cb2)

        self.key_sub = rospy.Subscriber(
            "keyboard",
            Vector3,
            self.key_cb)

        self.pix1 = rospy.Publisher(
            "pix1",
            Vector3)

        self.pix2 = rospy.Publisher(
            "pix2",
            Vector3)

        # Detection rate
        self.detection_rate = 10
        self.rate = rospy.Rate(self.detection_rate)

    def wii_cb1(self, data):
        self.wii1_detected = True
        detect_array = data.ir_tracking
        self.wii_pix1.x = detect_array[0].x
        self.wii_pix1.y = detect_array[0].y

    def wii_cb2(self, data):
        self.wii2_detected = True
        detect_array = data.ir_tracking
        self.wii_pix2.x = detect_array[0].x
        self.wii_pix2.y = detect_array[0].y

    def key_cb(self, data):
        # when down is pressed, pix stored
        if data.x == 1:
            self.key_pressed = True

    def run(self):

        while not (self.wii1_detected and self.wii2_detected):
            print("PointCalibration.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("PointCalibration.run() - Starting position calculation")

        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.key_pressed:
                self.key_pressed = False
                self.pix1.publish(self.wii_pix1)
                self.pix2.publish(self.wii_pix2)


if __name__ == "__main__":
    rospy.init_node("point_calibration")
    try:
        pointCalibration = PointCalibration()
        pointCalibration.run()
    except rospy.ROSInterruptException:
        pass
