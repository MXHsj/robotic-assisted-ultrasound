#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        arr = [0.1, 0.1, 0.1]
        data2send = Float64MultiArray()
        data2send.data = arr
        pub.publish(data2send)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
