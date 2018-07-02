#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from pynput.keyboard import Key, KeyCode, Listener

to_print = rospy.get_param("~print", False)
pub = rospy.Publisher('keyboard', Vector3, queue_size=1)
key_command = Vector3(0., 0., 0.)
start_listener = False
moveBindings = {
    Key.up: (-1, 0, 0),
    Key.down: (1, 0, 0),
    Key.left: (0, 1, 0),
    Key.right: (0, -1, 0),
    'w': (0, 0, -1),
    's': (0, 0, 1)
}

keys = [Key.up, Key.down, Key.left, Key.right, 'w', 's']


def on_press(key):

    global pub, key_command, start_listener, moveBindings, keys, to_print

    if type(key) is KeyCode:
        key = key.char

    if key == Key.f3:
        start_listener = True
        if to_print:
            print("Start key")
    elif (key in keys) and start_listener:
        if to_print:
            print("Good key")
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]

        key_command.x = key_command.x or x
        key_command.y = key_command.y or y
        key_command.z = key_command.z or z

        pub.publish(key_command)


def on_release(key):
    global pub, key_command, start_listener, moveBindings, keys, to_print
    try:
        if type(key) is KeyCode:
            key = key.char

        if key == Key.esc:
            # Stop listener
            start_listener = False
            print("ESC PRESSED")
            return False
        elif (key in keys) and start_listener:
            x = moveBindings[key][0]
            y = moveBindings[key][1]
            z = moveBindings[key][2]
            comm_list = [x, y, z]
            inv_comm_list = [(not i) for i in comm_list]

            key_command.x = np.sign(key_command.x) * (key_command.x and inv_comm_list[0])
            key_command.y = np.sign(key_command.y) * (key_command.y and inv_comm_list[1])
            key_command.z = np.sign(key_command.z) * (key_command.z and inv_comm_list[2])

            pub.publish(key_command)
            if to_print:
                print("Key released")
    except KeyError:
        pass


if __name__ == "__main__":

    global start_listener

    rospy.init_node('keyboard_listener')

    start_listener = False
    
    print("Keyboard listener started")
    # Collect events until released
    with Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()

    try:
        while not rospy.is_shutdown():
            pass
        listener.stop()
        print("Keyboard listener stopped")
    except rospy.ROSInterruptException:
        pass
