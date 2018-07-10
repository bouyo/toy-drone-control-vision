#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion
from wiimote.msg import State
from wiimote.msg import IrSourceInfo


class WiiDetection:

    def __init__(self):

        self.wii1_detected = False
        self.wii2_detected = False

        # NEURAL NETWORK CONSTANTS
        self.x_offset = np.array([15, 31, 64, 32])
        self.x_gain = np.array([0.00205549845837616, 0.0033167495854063, 0.00222222222222222, 0.00334448160535117])

        self.y_offset = np.array([-75, 79, 0])
        self.y_gain = np.array([0.0121951219512195, 0.027027027027027, 0.0666666666666667])

        # Layer 1
        self.b1 = np.array(
            [-3.6355608447828018, -0.37478230313700545, -0.28365517100965948, 0.13475925179971274, 0.12345034182560134,
             2.048057605462501, 2.2073576920513833, 0.56867066317350934])
        self.IW1_1 = np.array([[4.1748350906340397, -3.2631853112120033, -0.58331098450434704, -1.0752272479650751],
                          [1.1735931177016041, -0.93294031927571019, -1.4430537002579684, -0.26914138183361153],
                          [-0.11388486674129081, 0.057641196462793913, 1.2510735375148498, 0.074943426676298372],
                          [-0.50044938248853454, -0.50865164492642589, 0.5186203261889818, 0.34061728847158995],
                          [-1.3544537606133746, 0.57793663896364011, 1.245973501489779, 0.3195777254715827],
                          [-2.8777765994882669, -0.047375183963823113, 6.5595533967483961, 0.52332452189244216],
                          [-1.3024313160189775, 0.24942608549470482, 5.2583191907756204, 0.21293113718583509],
                          [-1.3031167441443161, 1.0420572303421776, 0.96436912373400441, -0.42009274529707108]])

        # Layer 2
        self.b2 = np.array([0.32367322349027206, -0.044989430932538252, -0.63309494832268998])
        self.LW2_1 = np.array([[-0.038678448131542899, -0.17886892060549772, 0.96594692808927485, -0.68962723007823057,
                           -0.38619815870059776, 1.9616296134453197, -1.7345597039146632, -0.41852499725389125],
                          [0.013726639873650937, -2.6961197186996286, 1.3845703827315146, -3.865549342410914,
                           -3.7979221525551745, -1.5287157311592205, 1.9763341179609812, 1.1438001999151957],
                          [-0.11255988909142788, 4.4589846055153322, 0.3103862125049473, -0.98175057088615003,
                           13.248050747190007, 1.2412420421959969, -1.5573678152922075, -4.3090812945330326]])

        # camera position in meters
        self.wii_1_pose = np.array([0.585, -1.68, 1.885])
        self.wii_2_pose = np.array([-0.585, -1.68, 1.885])

        # Needed variables
        self.wii_1_pixs = np.array([[-1., -1.],
                                   [-1., -1.],
                                   [-1., -1.]])
        self.wii_2_pixs = np.array([[-1., -1.],
                                   [-1., -1.],
                                   [-1., -1.]])

        self.detected_point = np.array([Vector3(0., 0., 0.),
                                        Vector3(0., 0., 0.),
                                        Vector3(0., 0., 0.)])
        self.drone_coord = Quaternion(0., 0., 0., 0.)

        # SUBS N PUBS
        self.wii_subscriber1 = rospy.Subscriber(
            "wiimote/state1",
            State,
            self.wii_cb1)

        self.wii_subscriber2 = rospy.Subscriber(
            "wiimote/state2",
            State,
            self.wii_cb2)

        self.drone_position = rospy.Publisher(
            'drone_position',
            Quaternion)

        # Detection rate
        self.detection_rate = 10
        self.rate = rospy.Rate(self.detection_rate)

    def wii_cb1(self, data):
        self.wii1_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 3):
            self.wii_1_pixs[i][0] = detect_array[i].x
            self.wii_1_pixs[i][1] = detect_array[i].y

    def wii_cb2(self, data):
        self.wii2_detected = True
        detect_array = data.ir_tracking
        for i in range(0, 3):
            self.wii_2_pixs[i][0] = detect_array[i].x
            self.wii_2_pixs[i][1] = detect_array[i].y

    def tansig(self, n):
        # neural network sigmoidal function
        return 2 / (1 + np.exp(-2 * n)) - 1

    def pix2coord(self, pix1, pix2):
        """
        Trained neural network. Gives world coordinate from camera pixels
        :param pix1: [x1, y1], wii1 pixel
        :param pix2: [x2, y2], wii2 pixel
        :return: point world coordinates
        """
        a1 = np.hstack((pix1, pix2))

        inputs = a1 - self.x_offset
        inputs = np.multiply(inputs, self.x_gain)
        inputs = inputs - 1

        a11 = np.dot(self.IW1_1, inputs)
        a12 = np.add(a11, self.b1)

        out1 = self.tansig(a12)

        a2 = np.dot(self.LW2_1, out1)
        a21 = np.add(a2, self.b2)

        out = a21 + 1
        out = np.divide(out, self.y_gain)
        out = out + self.y_offset

        return out

    def run(self):

        while not (self.wii1_detected and self.wii2_detected):
            print("WiiDetection.run() - Waiting for first measurement.")
            rospy.sleep(1)

        print("WiiDetection.run() - Starting position calculation")

        while not rospy.is_shutdown():
            self.rate.sleep()

            self.detected_point = np.array([Quaternion(-1., -1., -1., -np.pi/2),
                                            Quaternion(-1., -1., -1., -np.pi/2),
                                            Quaternion(-1., -1., -1., -np.pi/2)])

            point = np.array([[0., 0., 0.],
                              [0., 0., 0.],
                              [0., 0., 0.]])
            leds_detected = 0

            for i in range(0, 3):
                if self.wii_1_pixs[i][0] != -1:
                    if self.wii_2_pixs[i][0] != -1:
                        leds_detected += 1
                        point[i] = self.pix2coord(self.wii_1_pixs[i], self.wii_2_pixs[i])
                        self.detected_point[i].x = point[i][0]
                        self.detected_point[i].y = point[i][1]
                        self.detected_point[i].z = point[i][2]

            if leds_detected == 1:
                self.drone_coord = self.detected_point[0]
            elif leds_detected == 2:
                if self.detected_point[0].y < self.detected_point[1].y:
                    front_index = 0
                    back_index = 1
                else:
                    front_index = 1
                    back_index = 0
                self.drone_coord = self.detected_point[front_index]
                self.drone_coord.x = self.drone_coord.x - 0.05
                self.drone_coord.y = self.drone_coord.y + 0.1
                x1 = self.detected_point[front_index].x
                x2 = self.detected_point[back_index].x
                angle = -np.arccos((x1 - x2)/18)
                self.drone_coord.w = angle

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
