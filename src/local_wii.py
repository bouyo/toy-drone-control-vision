#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from wiimote.msg import State
from geometry_msgs.msg import Vector3
import csv
from visualization_msgs.msg import Marker
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from wiimote.msg import IrSourceInfo


class WiiLocal:

    def __init__(self):

        self.height = rospy.get_param("~h", "0")

        self.wii_detected = True
        self.data_stored = False

        # CONSTANT DEFINITIONS
        self.pix_width = 1024
        self.pix_height = 768

        # camera position in meters
        self.wii_1_pose = np.array([0., 0., 0.])

        self.wii_1_rotation = np.array([[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])

        self.pix1 = np.array([[]])
        self.pix2 = np.array([[]])
        self.pose = np.array([[]])

        # camera coord rotation (z-y-x)
        self.camera_1_rot = np.array([np.deg2rad(90), np.deg2rad(0), np.deg2rad(180)])

        # set the rotation matrix for each wii
        self.wii_1_rotation = self.set_camera(self.camera_1_rot)

        # Needed variables
        self.wii_1_pixs = np.array([-1, -1])

        # SUBS N PUBS
        self.wii_subscriber = rospy.Subscriber(
            "wiimote/state",
            State,
            self.wii_cb)

        self.drone_pos = rospy.Subscriber(
            "drone_position",
            Vector3,
            self.pose_cb)

        self.pix1_sub = rospy.Subscriber(
            "pix1",
            Vector3,
            self.pix1_cb)

        self.pix2_sub = rospy.Subscriber(
            "pix2",
            Vector3,
            self.pix2_cb)

        self.local_pose = rospy.Publisher(
            'local_position',
            Vector3)

        # Detection rate
        self.detection_rate = 2
        self.rate = rospy.Rate(self.detection_rate)

        self.csvfile_1 = "cam1_h" + str(self.height) + ".csv"
        self.csvfile_2 = "cam2_h" + str(self.height) + ".csv"
        self.csvfile_pose = "trajectory.csv"
        self.count = 0

    def pose_cb(self, data):
        self.count += 1
        if self.pose.size == 0:
            self.pose = np.array([data.x, data.y, data.z])
        else:
            self.pose = np.vstack((self.pose, [data.x, data.y, data.z]))

    def pix1_cb(self, data):
        self.count += 1
        if self.pix1.size == 0:
            self.pix1 = np.array([data.x, data.y])
        else:
            self.pix1 = np.vstack((self.pix1, [data.x, data.y]))

    def pix2_cb(self, data):
        self.count += 1
        if self.pix2.size == 0:
            self.pix2 = np.array([data.x, data.y])
        else:
            self.pix2 = np.vstack((self.pix2, [data.x, data.y]))

    def set_camera(self, rot):
        a = rot[0]
        b = rot[1]
        c = rot[2]
        Rz = np.array([[np.cos(c), -np.sin(c), 0.],
                       [np.sin(c),  np.cos(c), 0.],
                       [0.,         0.,        1.]])

        Ry = np.array([[np.cos(b),  0., np.sin(b)],
                       [0.,         1.,        0.],
                       [-np.sin(b), 0., np.cos(b)]])

        Rx = np.array([[1.,         0.,        0.],
                       [0.,  np.cos(a), -np.sin(a)],
                       [0.,  np.sin(a), np.cos(a)]])

        R = np.dot(Rz, np.dot(Ry, Rx))

        return R

    def wii_cb(self, data):
        self.wii_detected = True
        detect_array = data.ir_tracking
        self.wii_1_pixs[0] = detect_array[0].x
        self.wii_1_pixs[1] = detect_array[0].y

    def pix2vect(self, u, v):
        """
        Calculates the direction vector in camera coords

        :param u: x pixel
        :param v: y pixel
        :return: vector end  in camera local coords
        """

        alpha = np.deg2rad(((self.pix_width / 2) - u) * 0.043)
        beta = np.deg2rad((v - (self.pix_height / 2)) * 0.05)
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

        while not self.wii_detected:
            print("WiiDetection.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("WiiDetection.run() - Starting position calculation")

        while not rospy.is_shutdown():

            self.rate.sleep()
            """
            if self.count == 28:
                print(self.pix1)
                print(self.pix2)
                with open(self.csvfile_1, "w") as output:
                    writer = csv.writer(output, lineterminator='\n')
                    writer.writerows(self.pix1)

                with open(self.csvfile_2, "w") as output:
                    writer = csv.writer(output, lineterminator='\n')
                    writer.writerows(self.pix2)
            """
            if self.count > 30:
                rospy.sleep(1)
                with open(self.csvfile_pose, "w") as output:
                    writer = csv.writer(output, lineterminator='\n')
                    writer.writerows(self.pose)

            if self.wii_1_pixs[0] != -1 and self.data_stored:
                local_1 = self.pix2vect(self.wii_1_pixs[0], self.wii_1_pixs[1])
                glob_1 = self.vect2coord(self.wii_1_pose, self.wii_1_rotation, local_1)
                vector = Vector3(glob_1[0], glob_1[1], glob_1[2])
                self.local_pose.publish(vector)


if __name__ == "__main__":
    rospy.init_node("wii_local")
    try:
        wiiLocal = WiiLocal()
        wiiLocal.run()
    except rospy.ROSInterruptException:
        pass
