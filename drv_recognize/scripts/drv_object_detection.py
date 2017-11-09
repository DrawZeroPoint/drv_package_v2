#!/usr/bin/env python

# This is a upgraded version of drv_recognize.py, which increase the speed of detection
# to approximately 0.4s compared with former 1.4s on a Quadro K2200 GPU
# The main difference is that the Net only initialized for once

# Make sure these 2 lines are in your ~/.bashrc:

# export PYTHONPATH="/home/USER/py-faster-rcnn/lib:/home/USER/py-faster-rcnn/caffe-fast-rcnn/python:${PYTHONPATH}"
# export DRV=/home/USER/catkin_ws/src/drv_package

# Change 'USER' according to your environment

# Author: Zhipeng Dong
# 2017.11.9

# --------------------------------------------------------

import os
import sys
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
import numpy as np
import caffe
import rospy as rp
import drv_msgs.srv as srv
import drv_msgs.msg as msg
import cv2
from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray
from utils.timer import Timer
from cv_bridge import CvBridge

if "DRV" not in os.environ:
    print "Can't find environment variable DRV."
    sys.exit(1)

dir_prefix = os.environ['DRV'] + '/supplements/object_recognize/'
prototxt = dir_prefix + 'faster_rcnn_test.pt'
caffemodel = dir_prefix + 'VGG16_faster_rcnn_final.caffemodel'

if os.path.isfile(prototxt) and os.path.isfile(caffemodel):
    print 'Found Caffe prototxt and model.'
else:
    print 'Caffe prototxt or model not found!'
    sys.exit(2)

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'table', 'dog', 'horse',
           'motorbike', 'person', 'plant',
           'sheep', 'sofa', 'train', 'tv')


def load_net():
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    use_gpu = True
    if use_gpu:
        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
    else:
        caffe.set_mode_cpu()

    return caffe.Net(prototxt, caffemodel, caffe.TEST)


class FasterRCNN:
    def __init__(self, name, conf_thresh, nms_thresh):
        self._name = name
        self._init = False
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh

        rp.loginfo("Faster RCNN detector ready.")
        self._server = rp.Service(self._name, srv.recognize, self._callback)

    def _callback(self, req):
        if self._init is False:
            self._net = load_net()
            self._init = True

        use_gpu = True
        if use_gpu:
            caffe.set_mode_gpu()
            caffe.set_device(0)
            cfg.GPU_ID = 0
        else:
            caffe.set_mode_cpu()

        # Convert rosmsg to cv image
        # np_array = np.fromstring(req.img_in.data, np.uint8)
        # image = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(req.img_in, "bgr8")

        objects = self._detect(image)

        ro_msg = msg.recognized_objects()
        rsp = srv.recognizeResponse()

        for i in objects:
            bbox = i[0]
            class_name = i[2]

            label_str_msg = String()
            label_str_msg.data = class_name

            bbox_array_msg = UInt16MultiArray()
            for j in bbox:
                bbox_array_msg.data.append(j)

            ro_msg.labels.append(label_str_msg)
            ro_msg.bbox_arrays.append(bbox_array_msg)

        ro_msg.header = req.img_in.header

        rsp.obj_info = ro_msg
        return rsp

    def _detect(self, image):
        # Detect all object classes and regress object bounds
        timer = Timer()
        timer.tic()
        scores, boxes = im_detect(self._net, image)
        timer.toc()

        print ('Detection took {:.3f}s for '
               '{:d} object proposals').format(timer.total_time, boxes.shape[0])

        return self._post_process(scores, boxes)

    def _post_process(self, scores, boxes):
        obj_list = []
        for cls_ind, cls in enumerate(CLASSES[1:]):
            cls_ind += 1  # because we skipped background
            cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, self.nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= self.conf_thresh)[0]
            if len(inds) == 0:
                continue

            for i in inds:
                # Left up corner x, y; bottom down corner x, y
                bbox = dets[i, :4]
                score = dets[i, -1]

                instance = [bbox, score, cls]
                obj_list.append(instance)

        return obj_list


class NodeMain:
    def __init__(self):
        rp.init_node('drv_object_detector', anonymous=False)
        rp.on_shutdown(self.shutdown)
        conf_thresh = rp.get_param('conf_thresh', 0.8)
        node = FasterRCNN('drv_recognize', conf_thresh, 0.3)
        rp.spin()

    @staticmethod
    def shutdown():
        rp.loginfo("Shutting down")


if __name__ == '__main__':
    try:
        NodeMain()
    except rp.ROSInterruptException:
        rp.loginfo("Terminated")
