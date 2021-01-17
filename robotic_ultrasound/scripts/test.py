#! /usr/bin/env python3

import rospy
import copy
import numpy as np
from std_msgs.msg import Float64MultiArray
from cv2 import cv2

# T4x4 = np.array([[-1.0, -1.0, -1.0, -1.0], [-1.0, -1.0, -1.0, -1.0],
#                  [-1.0, -1.0, -1.0, -1.0], [-1.0, -1.0, -1.0, -1.0]])
T4x4 = -1*np.ones([4, 4])


def talker():
    pub = rospy.Publisher('cam_target', Float64MultiArray, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # arr = [-0.024, -0.999, 0.000, -0.999, 0.024, 0.027,
        arr = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0.5]    # fake T_cam_tar
        data2send = Float64MultiArray()
        data2send.data = arr
        pub.publish(data2send)
        rate.sleep()


# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass

# raw_data = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
# print(raw_data)
# raw_data.insert(2, 0)
# raw_data.insert(6, 0)
# raw_data.insert(11, 0)
# raw_data.insert(15, 1)
# print(raw_data)
# receiver = np.array([raw_data[0:4], raw_data[4:8],
#                      raw_data[8:12], raw_data[12:16]]).transpose()
# print(receiver)

def print_data():
    print("print data")


def othercase(argument):
    print("other case", argument)


def do_nothing():
    pass


def key_bindings(argument):
    switcher = {
        ord('p'): print_data,
        - 1: do_nothing
    }
    # Get the function from switcher dictionary
    func = switcher.get(argument, lambda: othercase(argument))
    # Execute the function
    func()


console = cv2.imread('RCM_key_inst.png')
while True:
    # cv2.namedWindow('user control', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('user control', console)
    key = cv2.waitKey(1)
    # print(key)
    if key & 0xFF == ord('q') or key == 27:
        print('quit')
        break
    else:
        key_bindings(key)
