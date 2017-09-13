#!/usr/bin/env python

import rospy
import cv2
from drv_msgs.srv import *
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


def call_recognize(img_msg):
    rospy.wait_for_service('drv_recognize')
    try:
        recognizer = rospy.ServiceProxy('drv_recognize', recognize)
        response = recognizer(img_msg)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def image_callback(img_msg):
    resp = call_recognize(img_msg)

    # rate = rospy.Rate(0.2)
    # rate.sleep()

    bg = CvBridge()
    cv_img_labeled = bg.imgmsg_to_cv2(resp.img_out, 'bgr8')

    print resp.obj_info.labels
    print resp.obj_info.box_array


def image_listener():
    rospy.init_node('recognize_client', anonymous=True)
    rospy.Subscriber('image_rect_color', CompressedImage, image_callback, queue_size=1, buff_size=921600)

    while not rospy.is_shutdown():
        rospy.spin()
        print 'reached'


if __name__ == "__main__":
    print "Requesting"
    image_listener()
