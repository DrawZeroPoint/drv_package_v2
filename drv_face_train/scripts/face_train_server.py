#!/usr/bin/env python

import roslib
import rospy
import process as ps

from drv_msgs.srv import *

roslib.load_manifest('drv_face_train')


def handle_train(req):
    rsp = face_trainResponse()
    ps.process()
    return rsp


def train_server():
    rospy.init_node('face_train_server')
    s = rospy.Service('face_train_service', face_train, handle_train)
    # The ~ before param id is mandatory, indicate that it is global
    i_n = rospy.get_param('~iter_num')
    ps.set_param(i_n)
    print "Ready to train face. The iteration number is %d", i_n
    rospy.spin()


if __name__ == "__main__":
    train_server()
