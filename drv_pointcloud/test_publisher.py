#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped


def talker():
    pub = rospy.Publisher('/vision/grasp/location', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.pose.position.x = 1
        pose.pose.position.y = -1
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
