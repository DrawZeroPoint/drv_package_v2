#!/usr/bin/env python

import roslib
import rospy
import cv2
import process as ps

from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from drv_msgs.msg import recognized_objects
from drv_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('drv_recognize')


def handle_recognize(req):
    rsp = recognizeResponse()

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(req.img_in, "bgr8")
        (rows, cols, channels) = cv_image.shape

        if cols != 640 or rows != 480:
            rospy.WARN('Can\'t get image.\n')
            return rsp
        else:
            result = ps.process(cv_image)
            ro_msg = recognized_objects()

            for i in result:
                bbox = i[0]
                score = i[1]
                class_name = i[2]

                cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 255), 2)
                cv2.putText(cv_image, '{:s} {:.3f}'.format(class_name, score),
                            (int(bbox[0]), int(bbox[1]) - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                label_str_msg = String()
                label_str_msg.data = class_name

                bbox_array_msg = UInt16MultiArray()
                for j in bbox:
                    bbox_array_msg.data.append(j)

                ro_msg.labels.append(label_str_msg)
                ro_msg.bbox_arrays.append(bbox_array_msg)

            ro_msg.header = req.img_in.header

            rsp.img_out = bridge.cv2_to_imgmsg(cv_image, "bgr8")
            rsp.obj_info = ro_msg
            return rsp

    except CvBridgeError as e:
        print(e)
        return rsp


def recognize_server():
    rospy.init_node('recognize_server')
    s = rospy.Service('drv_recognize', recognize, handle_recognize)
    print "Ready to recognize objects."
    rospy.spin()


if __name__ == "__main__":
    recognize_server()
