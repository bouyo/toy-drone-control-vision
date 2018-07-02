#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3


class WiiDetection:

    def __init__(self):

        self.started = False

        # CONSTANT DEFINITIONS
        self.pix_width = 1024
        self.pix_height = 768
        self.wii_1_pose = np.array([5, 0, 0])
        self.wii_2_pose = np.array([0, -5, 0])
        self.wii_1_rotation = np.array([[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])
        self.wii_2_rotation = np.array([[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])

        # camera coord rotation (z-y-x)
        self.camera_1_rot = np.array([np.pi/2, 0, -np.pi/2])
        self.camera_2_rot = np.array([np.pi/2, 0, 0])

        # set the rotation matrix for each wii
        self.wii_1_rotation = self.set_camera(self.camera_1_rot)
        self.wii_2_rotation = self.set_camera(self.camera_2_rot)

        # Needed variables
        self.wii_1_pix = np.array([0, 0])
        self.wii_2_pix = np.array([0, 0])
        self.detection_line_1 = np.array([0., 0., 0.])
        self.detection_line_2 = np.array([0., 0., 0.])
        self.detected_point = Vector3(0., 0., 0.)

        # SUBS N PUBS
        self.wii_subscriber = rospy.Subscriber(
            "wii/pose",
            Quaternion,
            self.wii_cb)

        self.drone_position = rospy.Publisher(
            'drone_position',
            Vector3)

        # Detection rate
        self.detection_rate = 5
        self.rate = rospy.Rate(self.detection_rate)

    def set_camera(self, rot):
        a = rot[0]
        b = rot[1]
        c = rot[2]
        Rz = np.array([[np.cos(c),  np.sin(c), 0.],
                      [-np.sin(c), np.cos(c), 0.],
                      [0.,         0.,        1.]])

        Ry = np.array([[np.cos(b),  0., -np.sin(b)],
                      [0.,         1.,        0.],
                      [np.sin(b),  0.,  np.cos(b)]])

        Rx = np.array([[1.,         0.,        0.],
                      [0.,  np.cos(a), np.sin(a)],
                      [0., -np.sin(a), np.cos(a)]])

        R = np.dot(Rz, np.dot(Ry, Rx))

        return R

    def wii_cb(self, data):
        self.started = True
        self.wii_1_pix[0] = data.x
        self.wii_1_pix[1] = data.y
        self.wii_2_pix[0] = data.z
        self.wii_2_pix[1] = data.w

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

        while not self.started:
            print("WiiDetection.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("WiiDetection.run() - Starting position calculation")

        while not rospy.is_shutdown():
            self.rate.sleep()

            local_1 = self.pix2vect(self.wii_1_pix[0], self.wii_1_pix[1])
            local_2 = self.pix2vect(self.wii_2_pix[0], self.wii_2_pix[1])

            global_1 = self.vect2coord(self.wii_1_pose, self.wii_1_rotation, local_1)
            global_2 = self.vect2coord(self.wii_2_pose, self.wii_2_rotation, local_2)

            point1, point2, _ = self.line_distance(self.wii_1_pose, global_1, self.wii_2_pose, global_2)

            self.detected_point.x = (point1[0] + point2[0]) / 2
            self.detected_point.y = (point1[1] + point2[1]) / 2
            self.detected_point.z = (point1[2] + point2[2]) / 2

            self.drone_position.publish(self.detected_point)


if __name__ == "__main__":
    rospy.init_node("wii_detection")
    try:
        wiiDetection = WiiDetection()
        wiiDetection.run()
    except rospy.ROSInterruptException:
        pass
