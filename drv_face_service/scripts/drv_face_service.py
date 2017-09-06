#!/usr/bin/env python

import roslib
import rospy
import process as ps

from drv_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('drv_face_service')


def handle_face_recognize(req):
    rsp = face_recognizeResponse()

    bridge = CvBridge()
    fl_msg = []
    try:
        for im in req.images_in:
            cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
            # Leave image preprocess to client
            result = ps.process(cv_image)

            if result[1] > 0.9:
                name_id_msg = result[0] + 1  # known face id start with 1
            else:
                name_id_msg = 0  # unknown face has id 0
            fl_msg.append(name_id_msg)

        rsp.face_label_ids = fl_msg
        return rsp

    except CvBridgeError as e:
        print(e)
        return rsp


def face_recognize_server():
    rospy.init_node('face_recognize_server')
    s = rospy.Service('drv_face_service', face_recognize, handle_face_recognize)

    print "Ready to recognize face."
    rospy.spin()


if __name__ == "__main__":
    face_recognize_server()
