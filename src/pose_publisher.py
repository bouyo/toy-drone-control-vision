#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import UInt16
from geometry_msgs.msg import Quaternion
from pynput.keyboard import Key, KeyCode, Listener

class Keyboard:

    def __init__(self):
        self.to_print = rospy.get_param("~print", False)
        self.pub = rospy.Publisher('drone/pose_ref', Quaternion, queue_size=1)
        self.pub1 = rospy.Publisher('keyboard', UInt16, queue_size=1)
        self.key_command = Quaternion(0., 0., 0., 0.)
        self.key_stop = UInt16
        self.old = [0., 0., 0., 0.]
        self.start_listener = False
        self.first_meas = True
        self.moveDrone = {
            'w': np.array([0, -1, 0, 0.]),
            's': np.array([0., 1., 0., 0.]),
            'a': np.array([1., 0., 0., 0.]),
            'd': np.array([-1., 0., 0., 0.]),
            '8': np.array([0., 0., 1., 0.]),
            '2': np.array([0., 0., -1., 0.]),
            '4': np.array([0., 0., 0., 1.]),
            '6': np.array([0., 0., 0., -1.]),
        }

        self.keys = ['w', 's', 'a', 'd', '8', '2', '4', '6']

        self.measurement_subscriber = rospy.Subscriber(
            "drone_position",
            Quaternion,
            self.measurement_cb)

    def measurement_cb(self, data):
        if self.first_meas:
            self.key_command = data
            self.first_meas = True

    def on_press(self, key):

        if type(key) is KeyCode:
            key = key.char

        if key == Key.f3:
            self.start_listener = True
            if self.to_print:
                print("Start key")
        elif (key == Key.down) and self.start_listener:
            if self.to_print:
                print("Good key - Stop drone")
            if self.key_stop == 0:
                self.key_stop = 1
                self.pub1.publish(self.key_stop)
        elif (key in self.keys) and self.start_listener:
            if self.to_print:
                print("Good key - Move drone")

            for i in range(0, 4):
                self.old[i] = np.add(self.old[i], self.moveDrone[key][i])

            self.pub.publish(Quaternion(self.old[0], self.old[1], self.old[2], self.old[3]))

    def on_release(self, key):

        try:
            if type(key) is KeyCode:
                key = key.char

            if key == Key.esc:
                # Stop listener
                self.start_listener = False
                print("ESC PRESSED")
                return False
            elif (key == Key.down) and self.start_listener:
                self.key_stop = 0
                self.pub1.publish(self.key_stop)
                if self.to_print:
                    print("Key released")

        except KeyError:
            pass

    def run(self):
        self.start_listener = False
        while not self.first_meas:
            print("DroneControl.run() - Waiting for first reference.")
            rospy.sleep(1)

        print("Keyboard listener started")
        # Collect events until released
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()

        while not rospy.is_shutdown():
            pass
        listener.stop()
        print("Keyboard listener stopped")

if __name__ == "__main__":
    rospy.init_node('keyboard_listener')

    try:
        control = Keyboard()
        control.run()
    except rospy.ROSInterruptException:
        pass
