#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from math import pow, atan2, sqrt


class Robot:
    def __init__(self):
        # Creating our node, publisher and subscriber
        rospy.init_node('go_to_goal', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber('/vision/grasp/location', PoseStamped, self.callback)
        self.pose = PoseStamped()
        self.distance_tolerance = 0.05
        # The actual frequency of publishing is determined by the lower one of the callback rate and sleep rate
        self.rate = rospy.Rate(30)
        self.hesitate_rate = rospy.Rate(1)
        # previous distance temp
        self.abs_temp_x = 100.0
        self.abs_temp_y = 100.0

        print 'Ready to go to goal with tolerance ', self.distance_tolerance

    # Callback function implementing the pose value received
    # This pose value is actually the offset value relevant to robot's current frame
    def callback(self, data):
        self.pose = data
        self.pose.pose.position.x = round(self.pose.pose.position.x, 3)
        self.pose.pose.position.y = round(self.pose.pose.position.y, 3)

        force_stop = False
        if not self.get_distance(self.pose.pose.position.x, self.pose.pose.position.y) > \
                self.get_distance(self.abs_temp_x, self.abs_temp_y):
            self.abs_temp_x = self.pose.pose.position.x
            self.abs_temp_y = self.pose.pose.position.y
        else:
            self.abs_temp_x = 100.0
            self.abs_temp_y = 100.0
            force_stop = True
            print 'The goal may be changed.'
            # if the distance is larger after adjustment, the target may be lost or wrong
            # so we need force the robot to stop
        self.move2goal(force_stop)

    @staticmethod
    def get_distance(goal_x, goal_y):
        distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2))
        return distance

    def move2goal(self, force_stop=False):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.pose.pose.position.x
        goal_pose.pose.position.y = self.pose.pose.position.y

        vel_msg = Twist()

        if force_stop:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            self.hesitate_rate.sleep()  # hesitate for resetting the goal
        else:
            if self.get_distance(goal_pose.pose.position.x, goal_pose.pose.position.y) >= self.distance_tolerance:
                # Proportional Controller
                # linear velocity in the x-axis:
                vel_msg.linear.x = 1.5 * sqrt(pow(goal_pose.pose.position.x, 2) +
                                              pow(goal_pose.pose.position.y, 2))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # angular velocity in the z-axis:
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                # notice y in base_link is actually x in arc tan
                vel_msg.angular.z = 4 * (atan2(goal_pose.pose.position.x, goal_pose.pose.position.y))

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()  # give this speed some time for execution
            else:
                # Stopping our robot after the movement is over
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                print 'The robot has reached the goal.'


if __name__ == '__main__':
    try:
        x = Robot()
        rospy.spin()  # the program will not stop as long as this node is ok

    except rospy.ROSInterruptException:
        pass
