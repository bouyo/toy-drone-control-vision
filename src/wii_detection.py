#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from wiimote.msg import State
from wiimote.msg import IrSourceInfo
from sensor_msgs.msg import Imu


class WiiDetection:

    def __init__(self):

        self.measured_angle = True

        self.wii1_detected = False
        self.wii2_detected = False

        # CONSTANT DEFINITIONS
        self.g = 9.80665
        self.pix_width = 1024
        self.pix_height = 768

        # camera position in meters
        self.wii_1_acc = Vector3(0., 0., 0.)
        self.wii_2_acc = Vector3(0., 0., 0.)
        self.wii_1_pose = np.array([0.585, -1.68, 1.885])
        self.wii_2_pose = np.array([-0.585, -1.68, 1.885])

        self.wii_1_rotation = np.array([[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])
        self.wii_2_rotation = np.array([[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])

        # camera coord rotation (z-y-x)
        self.camera_1_rot = np.array([np.deg2rad(132), np.deg2rad(20), np.deg2rad(180)])
        self.camera_2_rot = np.array([np.deg2rad(132), np.deg2rad(-20), np.deg2rad(180)])

        # set the rotation matrix for each wii
        self.wii_1_rotation = self.set_camera(self.camera_1_rot)
        self.wii_2_rotation = self.set_camera(self.camera_2_rot)

        # Needed variables
        self.wii_1_pixs = np.array([[-1., -1.],
                                   [-1., -1.],
                                   [-1., -1.]])
        self.wii_2_pixs = np.array([[-1., -1.],
                                   [-1., -1.],
                                   [-1., -1.]])
        self.detection_line_1 = np.array([[0., 0., 0.],
                                         [0., 0., 0.],
                                         [0., 0., 0.]])
        self.detection_line_2 = np.array([[0., 0., 0.],
                                         [0., 0., 0.],
                                         [0., 0., 0.]])
        self.detected_point = np.array([Vector3(0., 0., 0.),
                                        Vector3(0., 0., 0.),
                                        Vector3(0., 0., 0.)])
        self.drone_coord = Vector3(0., 0., 0.)

        # SUBS N PUBS
        self.wii_subscriber1 = rospy.Subscriber(
            "wiimote/state1",
            State,
            self.wii_cb1)

        self.wii_subscriber2 = rospy.Subscriber(
            "wiimote/state2",
            State,
            self.wii_cb2)

        self.wii_acceleration1 = rospy.Subscriber(
            "/imu/data1",
            Imu,
            self.wii_acc1)

        self.wii_acceleration2 = rospy.Subscriber(
            "/imu/data2",
            Imu,
            self.wii_acc2)

        self.drone_position = rospy.Publisher(
            'drone_position',
            Vector3)

        # Detection rate
        self.detection_rate = 2
        self.rate = rospy.Rate(self.detection_rate)

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

        Rx = np.array([[1.,         0.,         0.],
                       [0.,  np.cos(a), -np.sin(a)],
                       [0.,  np.sin(a),  np.cos(a)]])

        R = np.dot(Rz, np.dot(Ry, Rx))

        return R

    def wii_cb1(self, data):
        self.wii1_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 2):
            self.wii_1_pixs[i][0] = detect_array[i].x
            self.wii_1_pixs[i][1] = detect_array[i].y

    def wii_cb2(self, data):
        self.wii2_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 3):
            self.wii_2_pixs[i][0] = detect_array[i].x
            self.wii_2_pixs[i][1] = detect_array[i].y

    def wii_acc1(self, data):
        self.wii_1_acc = data.linear_acceleration

    def wii_acc2(self, data):
        self.wii_2_acc = data.linear_acceleration

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

        while not (self.wii1_detected and self.wii2_detected):
            print("WiiDetection.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("WiiDetection.run() - Starting position calculation")

        if self.measured_angle:

            self.camera_1_rot[0] = np.pi/2 + np.arcsin(self.wii_1_acc.y / self.g)
            self.camera_1_rot[2] = (np.pi * 3/2) - np.arccos(self.wii_1_acc.x / self.g)

            self.camera_2_rot[0] = np.pi/2 + np.arcsin(self.wii_2_acc.y / self.g)
            self.camera_2_rot[2] = (np.pi * 3/2) - np.arccos(self.wii_2_acc.x / self.g)

            self.wii_1_rotation = self.set_camera(self.camera_1_rot)
            self.wii_2_rotation = self.set_camera(self.camera_2_rot)

        while not rospy.is_shutdown():
            self.rate.sleep()

            local_1 = np.array([[0., 0., 0.],
                               [0., 0., 0.],
                               [0., 0., 0.]])

            local_2 = np.array([[0., 0., 0.],
                               [0., 0., 0.],
                               [0., 0., 0.]])

            global_1 = np.array([[0., 0., 0.],
                                [0., 0., 0.],
                                [0., 0., 0.]])

            global_2 = np.array([[0., 0., 0.],
                                [0., 0., 0.],
                                [0., 0., 0.]])

            self.detected_point = np.array([Vector3(0., 0., 0.),
                                            Vector3(0., 0., 0.),
                                            Vector3(0., 0., 0.)])

            point1 = np.array([[0., 0., 0.],
                              [0., 0., 0.],
                              [0., 0., 0.]])
            point2 = np.array([[0., 0., 0.],
                              [0., 0., 0.],
                              [0., 0., 0.]])
            distance = np.array([10., 10., 10.])
            found = np.array([False, False, False])
            leds_detected = 0

            for i in range(0, 3):
                if self.wii_1_pixs[i][0] != -1:
                    local_1[i] = self.pix2vect(self.wii_1_pixs[i][0], self.wii_1_pixs[i][1])
                    global_1[i] = self.vect2coord(self.wii_1_pose, self.wii_1_rotation, local_1[i])

                if self.wii_2_pixs[i][0] != -1:
                    local_2[i] = self.pix2vect(self.wii_2_pixs[i][0], self.wii_2_pixs[i][1])
                    global_2[i] = self.vect2coord(self.wii_2_pose, self.wii_2_rotation, local_2[i])

            for i in range(0, 3):
                if not np.any(global_1[i]):
                    break
                else:
                    distance = np.array([10., 10., 10.])
                    for j in range(0, 3):
                        if found[j]:
                            continue
                        else:
                            if not np.any(global_2[j]):
                                break
                            else:
                                print("Pozicija 1 kamera = {}\nPozicija 1 objekt = {}".format(self.wii_1_pose, global_1[i]))
                                print("Pozicija 2 kamera = {}\nPozicija 2 objekt = {}".format(self.wii_2_pose, global_2[j]))
                                point1[j], point2[j], distance[j] = \
                                    self.line_distance(self.wii_1_pose, global_1[i],
                                                       self.wii_2_pose, global_2[j])
                    index = distance.argmin()
                    print(index)
                    self.detected_point[i].x = (point1[index][0] + point2[index][0]) / 2
                    self.detected_point[i].y = (point1[index][1] + point2[index][1]) / 2
                    self.detected_point[i].z = (point1[index][2] + point2[index][2]) / 2
                    found[index] = True
                    leds_detected += 1

            if leds_detected == 1:
                self.drone_coord = self.detected_point[0]
            elif leds_detected == 2:
                if self.detected_point[0].y < self.detected_point[1].y:
                    front_index = 0
                else:
                    front_index = 1
                self.drone_coord = self.detected_point[front_index]
                self.drone_coord.x = self.drone_coord.x - 0.05
                self.drone_coord.y = self.drone_coord.y + 0.1
            elif leds_detected == 3:
                self.drone_coord = self.detected_point[0]
            else:
                self.drone_coord = Vector3(0., 0., 0.)

            self.drone_position.publish(self.drone_coord)


if __name__ == "__main__":
    rospy.init_node("wii_detection")
    try:
        wiiDetection = WiiDetection()
        wiiDetection.run()
    except rospy.ROSInterruptException:
        pass
