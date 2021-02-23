#!/usr/bin/env python3
import rospy
from timeit import default_timer as timer
from std_msgs.msg import Float64


class SpeedCalculator:

    def __init__(self):
        self.old_left_pos = 0.0
        self.old_time_left_pos = timer()

        self.left_pos = 0.0

        self.old_right_pos = 0.0
        self.old_time_right_pos = timer()

        self.right_pos = 0.0

        self.subLeft = rospy.Subscriber("/left_pos", Float64, self.leftSpeedCb)
        self.subRight = rospy.Subscriber(
            "/right_pos", Float64, self.rightspeedCb)

        self.pubLeft = rospy.Publisher("/my_vel_left", Float64)
        self.pubRight = rospy.Publisher("/my_vel_right", Float64)

    def leftSpeedCb(self, msg):
        dt = timer() - self.old_time_left_pos
        # print(dt)
        self.old_time_left_pos = timer()
        self.left_pos = msg.data
        self.left_speed = (self.left_pos - self.old_left_pos)/dt
        self.old_left_pos = self.left_pos
        msg_send = Float64()
        msg_send.data = self.left_speed
        self.pubLeft.publish(msg_send)

    def rightspeedCb(self, msg):
        dt = timer() - self.old_time_right_pos
        self.old_time_right_pos = timer()
        self.right_pos = msg.data
        self.right_speed = (self.right_pos - self.old_right_pos)/dt
        self.old_right_pos = self.right_pos
        msg_send = Float64()
        msg_send.data = self.right_speed
        self.pubRight.publish(msg_send)


if __name__ == "__main__":
    rospy.init_node('vel_calc')
    s = SpeedCalculator()
    rospy.spin()
