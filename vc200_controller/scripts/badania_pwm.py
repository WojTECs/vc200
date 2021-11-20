#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import  Twist

def talker():
    cmdvel = rospy.Publisher('/diff_drive/cmd_vel', Twist, queue_size=1)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    i = int()
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        cmdvel_msg = Twist()

        cmdvel_msg.linear.x = 0.7
        cmdvel_msg.angular.z = 0

        if (i > 28) :
            cmdvel_msg.linear.x = 0.52
            cmdvel_msg.angular.z = -1.6
        

        if (i > 200) :
            cmdvel_msg.linear.x = 0.3
            cmdvel_msg.angular.z = -1.6

        if (i > 500) :
            cmdvel_msg.linear.x = -0.3
            cmdvel_msg.angular.z = 0

        if (i > 550) :
            cmdvel_msg.linear.x = 0.0
            cmdvel_msg.angular.z = 0.0
        

        cmdvel.publish(cmdvel_msg)

        if (i > 560) : 
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass