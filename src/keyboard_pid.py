#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt16
from pynput.keyboard import Key, KeyCode, Listener

to_print = rospy.get_param("~print", False)
pub = rospy.Publisher('keyboard', UInt16, queue_size=1)
key_command = UInt16
start_listener = False


def on_press(key):
    global pub, key_command, start_listener, to_print

    if type(key) is KeyCode:
        key = key.char

    if key == Key.f3:
        start_listener = True
        if to_print:
            print("Start key")
    elif (key == Key.down) and start_listener:
        if to_print:
            print("Good key")
        if key_command == 0:
            key_command = 1

            pub.publish(key_command)


def on_release(key):
    global pub, key_command, start_listener, to_print
    try:
        if type(key) is KeyCode:
            key = key.char

        if key == Key.esc:
            # Stop listener
            start_listener = False
            print("ESC PRESSED")
            return False
        elif (key == Key.down) and start_listener:
            key_command = 0
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
