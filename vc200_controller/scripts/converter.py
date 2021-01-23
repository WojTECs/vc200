#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("converter")

    left_vel = rospy.Publisher("/left_vel", Float64, queue_size=1)
    right_vel = rospy.Publisher("/right_vel", Float64, queue_size=1)

    left_pos = rospy.Publisher("/left_pos", Float64, queue_size=1)
    right_pos = rospy.Publisher("/right_pos", Float64, queue_size=1)

    left_effort = rospy.Publisher("/left_effort", Float64, queue_size=1)
    right_effort = rospy.Publisher("/right_effort", Float64, queue_size=1)

    def cb(msg):
        # rospy.loginfo("pub")
        left_vel.publish(msg.velocity[0])
        right_vel.publish(msg.velocity[1])
        left_pos.publish(msg.position[0])
        right_pos.publish(msg.position[1])
        left_effort.publish(msg.effort[0])
        right_effort.publish(msg.effort[1])


    rospy.Subscriber("joint_states", JointState, callback=cb, queue_size=1)

    rospy.spin()
