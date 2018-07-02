#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from pixy import *
from ctypes import *
from geometry_msgs.msg import Vector3


class Blocks (Structure):
    fields_ = [("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint)]


class PixyNode:

    def __init__(self, id):

        pixy_init()

        self.pixy_id = id + 1

        print(self.pixy_id)

        rospy.set_param('~pixy_id', self.pixy_id)

        self.blocks = BlockArray(2)

        self.pixel = Vector3(0., 0., 0.)

        self.pixy_pix = rospy.Publisher('pixy_pix' + str(self.pixy_id),
                                        Vector3,
                                        queue_size=5)

        # Detection rate
        self.detection_rate = 50
        self.rate = rospy.Rate(self.detection_rate)

    def run(self):

        while not rospy.is_shutdown():
            self.rate.sleep()

            count = 0
            try:
                count = pixy_get_blocks(100, self.blocks)
            except:
                print("PixyNode.run() - Could not read from pixy")

            if count > 0:
                # Blocks found
                self.pixel.x = self.blocks[0].x
                self.pixel.y = self.blocks[0].y

                self.pixy_pix.publish(self.pixel)


if __name__ == "__main__":

    rospy.init_node("pixy")
    id = rospy.get_param('~pixy_id', 0)
    try:
        pixy_node = PixyNode(id)
        pixy_node.run()
    except rospy.ROSInterruptException:
        pass
