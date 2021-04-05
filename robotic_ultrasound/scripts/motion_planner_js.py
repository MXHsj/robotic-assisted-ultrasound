#! /usr/bin/env python3
import math
import copy
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState


class Teleop:
    def __init__(self):
        rospy.init_node('franka_teleop_joy')

        # self.deadman_button = rospy.get_param('~deadman_button', 0)
        self.cmd = None
        self.T_O_ee = np.array([[-0.0117, -0.9996, 0.0239, 0.0],
                                [-0.9989, 0.01278, 0.0435, 0.0],
                                [-0.0438, -0.0234, -0.9987, 0.0],
                                [0.3439, 0.0005, 0.4420, 1.0]]).transpose()
        self.curr_slave = Twist()
        self.curr_master = Twist()
        self.last_slave = Twist()
        self.last_master = Twist()
        self.d_master = Twist()
        self.d_slave = Twist()

        cmd_pub = rospy.Publisher('cmd_js', Twist, queue_size=1)
        rospy.Subscriber("joy", Joy, self.js_callback)
        rospy.Subscriber('franka_state_controller/franka_states',
                         FrankaState, self.ee_callback)

        freq = rospy.get_param('~hz', 100)
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            # update slave state
            eul = self.rot2rpy(self.T_O_ee[:3, :3])
            self.curr_slave.linear.x = self.T_O_ee[0, 3]
            self.curr_slave.linear.y = self.T_O_ee[1, 3]
            self.curr_slave.linear.z = self.T_O_ee[2, 3]
            self.curr_slave.angular.x = eul[0]
            self.curr_slave.angular.y = eul[1]
            self.curr_slave.angular.z = eul[2]
            # calculate slave velocity
            self.d_slave.linear.x = \
                (self.curr_slave.linear.x-self.last_slave.linear.x)/freq
            self.d_slave.linear.y = \
                (self.curr_slave.linear.y-self.last_slave.linear.y)/freq
            self.d_slave.linear.z = \
                (self.curr_slave.linear.z-self.last_slave.linear.z)/freq
            self.d_slave.angular.x = \
                (self.curr_slave.angular.x-self.last_slave.angular.x)/freq
            self.d_slave.angular.y = \
                (self.curr_slave.angular.y-self.last_slave.angular.y)/freq
            self.d_slave.angular.z = \
                (self.curr_slave.angular.z-self.last_slave.angular.z)/freq
            # calculate master velocity
            self.d_master.linear.x = \
                (self.curr_master.linear.x-self.last_master.linear.x)/freq
            self.d_master.linear.y = \
                (self.curr_master.linear.y-self.last_master.linear.y)/freq
            self.d_master.linear.z = \
                (self.curr_master.linear.z-self.last_master.linear.z)/freq
            self.d_master.angular.x = \
                (self.curr_master.angular.x-self.last_master.angular.x)/freq
            self.d_master.angular.y = \
                (self.curr_master.angular.y-self.last_master.angular.y)/freq
            self.d_master.angular.z = \
                (self.curr_master.angular.z-self.last_master.angular.z)/freq
            # publish command
            if self.cmd:
                cmd_pub.publish(self.cmd)
            self.last_slave = copy.copy(self.curr_slave)
            rate.sleep()

    def map2motion(self, data):
        '''map joystick input to desired eef pose'''
        # TODO: input filtering
        temp = Twist()
        linxscale = 0.05  # meter
        linyscale = 0.05
        if data.buttons[0] == 1:    # control angular
            temp.angular.x = data.axes[0]
            temp.angular.y = data.axes[1]
            temp.angular.z = data.axes[4]
        else:                       # control linear
            temp.linear.x = self.curr_slave.linear.x+linxscale*data.axes[1]
            temp.linear.y = self.curr_slave.linear.y+linyscale*data.axes[0]
        return temp

    def js_callback(self, js_msg):
        '''acceleration command: u = Kp*(Xm-Xs) + Kd*(dXm-dXs)'''
        self.curr_master = self.map2motion(js_msg)
        cmd = Twist()
        # linear
        cmd.linear.x = \
            0.005*(self.curr_master.linear.x - self.curr_slave.linear.x) + \
            0.001*(self.d_master.linear.x - self.d_slave.linear.x)
        cmd.linear.y = \
            0.005*(self.curr_master.linear.y - self.curr_slave.linear.y) + \
            0.001*(self.d_master.linear.y - self.d_slave.linear.y)
        # angular
        cmd.angular.x = \
            0.005*(self.curr_master.angular.x - self.curr_slave.angular.x) + \
            0.001*(self.d_master.angular.x - self.d_slave.angular.x)
        cmd.angular.x = \
            0.005*(self.curr_master.angular.y - self.curr_slave.angular.y) + \
            0.001*(self.d_master.angular.y - self.d_slave.angular.y)
        cmd.angular.x = \
            0.005*(self.curr_master.angular.z - self.curr_slave.angular.z) + \
            0.001*(self.d_master.angular.z - self.d_slave.angular.z)
        self.cmd = cmd

    def ee_callback(self, ee_msg):
        EE_pos = ee_msg.O_T_EE_d  # inv 4x4 matrix
        self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                                EE_pos[12:16]]).transpose()

    def rot2rpy(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])


if __name__ == "__main__":
    Teleop()
