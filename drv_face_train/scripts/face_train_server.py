#!/usr/bin/env python

import roslib
import rospy
import process as ps

from drv_msgs.srv import *

roslib.load_manifest('drv_face_train')


def handle_train(req):
    rsp = face_trainResponse

    result = ps.process()

    rsp.accuracy = result
    return rsp


def train_server():
    rospy.init_node('face_train_server')
    s = rospy.Service('face_train_service', face_train, handle_train)
    print "Ready to train face."
    rospy.spin()


if __name__ == "__main__":
    train_server()
