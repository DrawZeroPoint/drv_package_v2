#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32

from drv_msgs.srv import *

pubSR = rospy.Publisher('/comm/msg/vision/select_request', UInt32, queue_size=1)
pubInfo = rospy.Publisher('/comm/msg/vision/info', String, queue_size=1)

# This is used for getting selection here and in JARVIS
param_vision_search_param_lock = '/vision/select/param/lock'

# If true, will select the target if only 1 candidate present
fast_select = True


def handle_user_select(req):
    param_control_target_is_set = '/comm/param/control/target/is_set'
    param_control_user_selected = '/comm/param/control/target/selected'

    rsp = user_selectResponse()
    select_msg = String()

    num = req.select_num

    if num == 1 and fast_select:
        rsp.selected_id = 1
        select_msg.data = 'Auto selected target No.1.'
        print select_msg.data
        pubInfo.publish(select_msg)
        return rsp

    selected = -1

    info = "Please choose the one you want, if no target, enter 0."
    print info
    info_msg = String()
    info_msg.data = "CHOOSE"
    pubInfo.publish(info_msg)  # Send this msg to APP for display info

    # Broadcast the target select request
    sr_msg = UInt32()
    sr_msg.data = num
    pubSR.publish(sr_msg)

    rospy.set_param(param_vision_search_param_lock, False)  # unlock param change
    while selected < 0:
        if rospy.has_param(param_control_target_is_set):
            if not rospy.get_param(param_control_target_is_set):
                selected = 0
                break
        if rospy.has_param(param_control_user_selected):
            selected = rospy.get_param(param_control_user_selected)
            if num >= selected > 0:
                select_msg.data = 'User has selected target No.' + str(selected)
                print select_msg.data
                pubInfo.publish(select_msg)
                break
            elif selected == 0:
                select_msg.data = 'No target selected.'
                print select_msg.data
                pubInfo.publish(select_msg)
                break
            else:
                selected = -1

    rsp.selected_id = int(selected)
    # Reset the select parameter for next selection
    rospy.set_param(param_control_user_selected, -1)
    rospy.set_param(param_vision_search_param_lock, True)

    return rsp


def user_select_server():
    rospy.init_node('user_select_server')
    s = rospy.Service('drv_user', user_select, handle_user_select)
    global fast_select
    fast_select = rospy.get_param('fast_select', True)
    rospy.set_param(param_vision_search_param_lock, True)  # at beginning set param change locked
    print "Ready to receive user selection."
    rospy.spin()


if __name__ == "__main__":
    user_select_server()
