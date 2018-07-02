#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16


class DroneControl:

    def __init__(self):

        self.change = [0, 0, 0]

        # key pressed speed increment
        self.dx = 3

        # max speed
        self.max_thrust = 179
        self.max_side = 179
        self.min_speed = 0
        self.mid_speed = 92

        self.ref_set = False
        self.speed = 0
        self.pose_ref = Quaternion(0., 0., 0., 0.)
        self.wii_pose = Vector3(0., 0., 0.)
        self.key_speed = Vector3(self.mid_speed, self.mid_speed, self.max_thrust)

        # initialize subscribers
        self.pose_subscriber = rospy.Subscriber(
            "drone/pose_ref",
            Quaternion,
            self.setpoint_cb)

        self.key_subscriber = rospy.Subscriber(
            "keyboard",
            Vector3,
            self.keyboard_cb)

        self.wii_subscriber = rospy.Subscriber(
            "wii/pose",
            Vector3,
            self.wii_cb)

        self.drone_input = rospy.Publisher(
            'control/drone_input',
            Vector3)

        # initialize publishers
        self.control_value = rospy.Publisher(
            'control_value',
            Quaternion,
            queue_size=10)

        self.toggle_speed = rospy.Publisher(
            'toggle_speed',
            UInt16)

        self.u = Quaternion(0., 0., 0., 0.)

        # Controller rate
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)

    def setpoint_cb(self, data):

        self.pose_ref = data
        self.speed = data.x
        self.ref_set = True

    def keyboard_cb(self, data):
        self.ref_set = True
        print("From keyboard:\nx = {}\ny = {}\nz = {}".format(data.x, data.y, data.z))
        self.change = [data.x * self.dx, data.y * self.dx, data.z * self.dx]

    def wii_cb(self, data):

        self.wii_pose.x = data.x
        self.wii_pose.y = data.y
        self.wii_pose.z = data.z

    def controller_dron_comm(self):
        self.drone_input.publish(Vector3(self.mid_speed, self.mid_speed, self.mid_speed))
        rospy.sleep(1)
        self.drone_input.publish(Vector3(self.mid_speed, self.mid_speed, self.max_thrust))
        rospy.sleep(1)

    def run(self):
        """ Run ROS node - computes PID algorithms for z and vz control """

        while not self.ref_set:
            print("DroneControl.run() - Waiting for first reference.")
            rospy.sleep(1)

        print("DroneControl.run() - Starting position control")

        self.controller_dron_comm()
        while not rospy.is_shutdown():
            self.rate.sleep()

            old = [self.key_speed.x, self.key_speed.y, self.key_speed.z]
            for i in range(0, 2):
                if not self.change[i]:
                    old[i] = self.mid_speed
                else:
                    old[i] = old[i] + self.change[i]
                    if old[i] > self.max_side:
                        old[i] = self.max_side
                    elif old[i] < self.min_speed:
                        old[i] = self.min_speed
            old[2] = old[2] + self.change[2]
            if old[2] < self.min_speed:
                old[2] = self.min_speed
            elif old[2] > self.max_thrust:
                old[2] = self.max_thrust

            self.key_speed.x = old[0]
            self.key_speed.y = old[1]
            self.key_speed.z = old[2]

            self.drone_input.publish(self.key_speed)
            self.toggle_speed.publish(self.speed)


if __name__ == "__main__":
    rospy.init_node("drone_control")
    try:
        control = DroneControl()
        control.run()
    except rospy.ROSInterruptException:
        pass
