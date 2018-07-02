#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from wiimote.msg import State
from wiimote.msg import IrSourceInfo


class WiiNode:

    def __init__(self):

        self.wii1_detected = False
        self.wii2_detected = False

        # Needed variables
        self.wii_1_pixs = np.array([[-1, -1, -1],
                                    [-1, -1, -1],
                                    [-1, -1, -1]])
        self.wii_2_pixs = np.array([[-1, -1, -1],
                                    [-1, -1, -1],
                                    [-1, -1, -1]])
        self.detection_line_1 = np.array([0., 0., 0.])
        self.detection_line_2 = np.array([0., 0., 0.])
        self.detected_point = Vector3(0., 0., 0.)

        # SUBS N PUBS
        self.wii_subscriber1 = rospy.Subscriber(
            "wiimote/state1",
            State,
            self.wii_cb1)

        self.wii_subscriber2 = rospy.Subscriber(
            "wiimote/state2",
            State,
            self.wii_cb2)

        self.drone_position1 = rospy.Publisher(
            'drone_position1',
            IrSourceInfo)

        self.drone_position2 = rospy.Publisher(
            'drone_position2',
            IrSourceInfo)

        # Detection rate
        self.detection_rate = 1
        self.rate = rospy.Rate(self.detection_rate)

    def wii_cb1(self, data):
        self.wii1_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 2):
            self.wii_1_pixs[i][0] = detect_array[i].x
            self.wii_1_pixs[i][1] = detect_array[i].y
            self.wii_1_pixs[i][2] = detect_array[i].ir_size

    def wii_cb2(self, data):
        self.wii2_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 2):
            self.wii_2_pixs[i][0] = detect_array[i].x
            self.wii_2_pixs[i][1] = detect_array[i].y
            self.wii_2_pixs[i][2] = detect_array[i].ir_size

    def pix2vect(self, u, v):
        """
        Calculates the direction vector in camera coords

        :param u: x pixel
        :param v: y pixel
        :return: vector end  in camera local coords
        """

        alpha = np.deg2rad(((self.pix_width / 2) - u) * 0.04)
        beta = np.deg2rad(((self.pix_height / 2) - v) * 0.04)
        x = np.sin(alpha) * np.cos(beta)
        z = np.cos(alpha) * np.cos(beta)
        y = np.sin(beta)

        return np.array([x, y, z])

    def vect2coord(self, pose, rot, vect):
        """
        Returns the end point of the vector in global coords

        :param pose: position of the camera
        :param rot: rotation matrix of the camera
        :param vect: vector in camera coords
        :return: coord: vector end point
        """

        rot_vect = np.dot(rot, vect)
        coord = pose + rot_vect
        return coord

    def line_distance(self, a0, a1, b0, b1):

        # Calculate denomitator
        A = a1 - a0
        B = b1 - b0
        magA = np.linalg.norm(A)
        magB = np.linalg.norm(B)

        _A = A / magA
        _B = B / magB

        cross = np.cross(_A, _B)
        denom = np.linalg.norm(cross) ** 2

        # Lines criss-cross: Calculate the projected closest points
        t = (b0 - a0)
        detA = np.linalg.det([t, _B, cross])
        detB = np.linalg.det([t, _A, cross])

        t0 = detA / denom
        t1 = detB / denom

        pA = a0 + (_A * t0)  # Projected closest point on segment A
        pB = b0 + (_B * t1)  # Projected closest point on segment B

        return pA, pB, np.linalg.norm(pA - pB)

    def run(self):

        while not (self.wii1_detected and self.wii2_detected):
            print("WiiDetection.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("WiiDetection.run() - Starting position calculation")

        while not rospy.is_shutdown():
            self.rate.sleep()

            ir_1 = [IrSourceInfo() for i in range(0, 2)]
            ir_2 = [IrSourceInfo() for i in range(0, 2)]
            for i in range(0, 2):
                ir_1[i].x = self.wii_1_pixs[i][0]
                ir_1[i].y = self.wii_1_pixs[i][1]
                ir_1[i].ir_size = self.wii_1_pixs[i][2]

                ir_2[i].x = self.wii_2_pixs[i][0]
                ir_2[i].y = self.wii_2_pixs[i][1]
                ir_2[i].ir_size = self.wii_2_pixs[i][2]

            self.drone_position1.publish(ir_1[0])
            self.drone_position2.publish(ir_2[0])


if __name__ == "__main__":
    rospy.init_node("wii_node")
    try:
        wiiDetection = WiiNode()
        wiiDetection.run()
    except rospy.ROSInterruptException:
        pass
