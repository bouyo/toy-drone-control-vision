#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16
from pid import PID
from datetime import datetime


class DroneControl:

    def __init__(self):

        # max speed
        self.max_thrust = 179.0
        self.max_side = 179.0
        self.min_speed = 0.0
        self.mid_speed = 92.0

        self.u = Quaternion(self.mid_speed, self.mid_speed, self.min_speed, self.mid_speed)

        self.pid_x = PID(1, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)
        self.pid_y = PID(1, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)
        self.pid_z = PID(1, 0, 0, self.max_thrust - self.min_speed, self.min_speed)
        self.pid_yaw = PID(1, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)

        self.ref_set = False
        self.pose_ref = Quaternion(0., 0., 0., math.pi/2)
        self.pose_meas = Quaternion(-1., -1., -1., -1.)

        # initialize subscribers
        self.pose_subscriber = rospy.Subscriber(
            "drone/pose_ref",
            Quaternion,
            self.setpoint_cb)

        self.measurement_subscriber = rospy.Subscriber(
            "drone_position",
            Quaternion,
            self.measurement_cb)

        self.drone_input = rospy.Publisher(
            'control/drone_input',
            Quaternion)

        # initialize publishers
        self.control_value = rospy.Publisher(
            'control_value',
            Quaternion,
            queue_size=10)

        self.error_pub = rospy.Publisher(
            'pose_error',
            Float64,
            queue_size=10)

        # Controller rate
        self.controller_rate = 10
        self.rate = rospy.Rate(self.controller_rate)
        self.controller_info = rospy.get_param("~verbose", False)

    def setpoint_cb(self, data):
        self.pose_ref = data
        self.ref_set = True

    def measurement_cb(self, data):
        self.pose_meas = data

    def run(self):
        """ Run ROS node - computes PID algorithms for control """

        while not self.ref_set:
            print("DroneControl.run() - Waiting for first reference.")
            rospy.sleep(1)

        print("DroneControl.run() - Starting position control")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            self.rate.sleep()

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()

            if dt < 0.99 / self.controller_rate:
                continue
            elif self.pose_meas.z == -1.0:
                continue

            self.t_old = t

            # HEIGHT CONTROL
            self.u.z = self.pid_x.compute(self.pose_ref.x, self.pose_meas.x, dt)

            # PITCH CONTROL OUTER LOOP
            # x - position control
            self.u.x = self.pid_x.compute(self.pose_ref.x, self.pose_meas.x, dt) + self.mid_speed

            # ROLL CONTROL OUTER LOOP
            # y position control
            self.u.y = self.pid_y.compute(self.pose_ref.y, self.pose_meas.y, dt) + self.mid_speed

            # PITCH AND ROLL YAW ADJUSTMENT
            roll_sp_2 = math.cos(self.pose_meas.w) * self.u.x + \
                        math.sin(self.pose_meas.w) * self.u.y
            self.u.y = math.cos(self.pose_meas.w) * self.u.y - \
                       math.sin(self.pose_meas.w) * self.u.x
            self.u.x = roll_sp_2

            # YAW CONTROL
            #error_yrc = self.pose_ref.w - self.pose_meas.w
            #if math.fabs(error_yrc) > math.pi:
            #    self.pose_ref.w = (self.pose_meas.w / math.fabs(self.pose_meas.w)) * \
            #                      (2 * math.pi - math.fabs(self.pose_ref.w))
            self.u.w = self.pid_yaw.compute(self.pose_ref.w, self.pose_meas.w, dt)

            # Calculate position error
            error = math.sqrt((self.pose_ref.x - self.pose_meas.x) ** 2 +
                              (self.pose_ref.y - self.pose_meas.y) ** 2 +
                              (self.pose_ref.z - self.pose_meas.z) ** 2)
            self.error_pub.publish(error)

            # Print out controller information
            if self.controller_info:
                print(dt)
                print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}\nyaw:{}\nyaw_m:{}".format(
                    self.pose_ref.x,
                    self.pose_meas.x,
                    self.pose_ref.y,
                    self.pose_meas.y,
                    self.pose_ref.z,
                    self.pose_meas.z,
                    self.pose_ref.w,
                    self.pose_meas.w))
                print("Current quadcopter height is: {}".format(self.pose_meas.z))
                print("Pitch PID output is:{}\n"
                      "Roll PID output is:{}\n"
                      "Yaw PID output is:{}\n"
                      "Error: {}\n".format(self.u.x, self.u.y, self.u.w, error))

            self.drone_input.publish(self.u)



if __name__ == "__main__":
    rospy.init_node("drone_control")
    try:
        control = DroneControl()
        control.run()
    except rospy.ROSInterruptException:
        pass