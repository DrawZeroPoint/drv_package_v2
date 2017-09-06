#!/usr/bin/env python

# Make sure these 2 lines are in your ~/.bashrc:

# export PYTHONPATH="/home/USER/py-faster-rcnn/lib:/home/USER/py-faster-rcnn/caffe-fast-rcnn/python:${PYTHONPATH}"
# export DRV=/home/USER/catkin_ws/src/drv_package

# Change 'USER' according to your environment

# Author: Zhipeng Dong
# 2017.9.6

# --------------------------------------------------------

import os
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
import numpy as np
import caffe

CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'table', 'dog', 'horse',
           'motorbike', 'person', 'plant',
           'sheep', 'sofa', 'train', 'tv')


def process(im):
    result = []

    cfg.TEST.HAS_RPN = True  # Use RPN for proposals
    if "DRV" not in os.environ:
        print "Can't find environment variable DRV."
        return result

    dir_prefix = os.environ['DRV']
    prototxt = dir_prefix + '/supplements/object_recognize/faster_rcnn_test.pt'
    caffemodel = dir_prefix + '/supplements/object_recognize/VGG16_faster_rcnn_final.caffemodel'

    if os.path.isfile(prototxt) and os.path.isfile(caffemodel):
        print 'Caffe prototxt and model found.'
    else:
        print 'Caffe prototxt or model not found!'
        return result

    use_gpu = True
    if use_gpu:
        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
    else:
        caffe.set_mode_cpu()
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    scores, boxes = im_detect(net, im)

    conf_thresh = 0.8
    nms_thresh = 0.3

    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1  # because we skipped background
        cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes, cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, nms_thresh)
        dets = dets[keep, :]

        inds = np.where(dets[:, -1] >= conf_thresh)[0]

        for i in inds:
            # left up corner x, y; bottom down corner x, y
            bbox = dets[i, :4]
            score = dets[i, -1]

            instance = [bbox, score, cls]
            result.append(instance)
            # result += str(int(bbox[0])) + ',' + str(int(bbox[1])) + ',' + str(int(bbox[2])) + ',' + str(
            #     int(bbox[3])) + ',' + str(cls) + ','

    print result
    return result
