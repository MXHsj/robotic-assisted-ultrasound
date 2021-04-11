#! /usr/bin/env python3
from logging import setLogRecordFactory
import math
import copy
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped


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
        self.wrench_slave = Wrench()
        self.d_force = 5.0  # max contact force [N]
        self.isContact = False

        cmd_pub = rospy.Publisher('cmd_js', Twist, queue_size=1)
        rospy.Subscriber("joy", Joy, self.js_callback)
        rospy.Subscriber('franka_state_controller/franka_states',
                         FrankaState, self.ee_callback)
        rospy.Subscriber('/franka_state_controller/F_ext',
                         WrenchStamped, self.force_callback)

        freq = rospy.get_param('~hz', 100)
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            # update slave state
            self.curr_slave.linear.x = self.T_O_ee[0, 3]
            self.curr_slave.linear.y = self.T_O_ee[1, 3]
            self.curr_slave.linear.z = self.T_O_ee[2, 3]
            eul = self.rot2rpy(self.T_O_ee[:3, :3])
            eul[0] = eul[0] + 6.28 if eul[0] < 0 else eul[0]    # -pi->pi
            # print(eul[0])
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
            self.last_master = copy.copy(self.curr_master)
            rate.sleep()

    def map2motion(self, data):
        '''map joystick input to desired eef pose'''
        temp = Twist()
        linxscale = 0.008     # meter
        linyscale = 0.008
        angxscale = 0.01      # radius
        angyscale = 0.01
        angzscale = 0.01
        Vz = self.T_O_ee[2, :3]		# approach vector
        # switch between contact & non-contact mode
        if data.buttons[4]:
            self.isContact = not self.isContact
            print("Contact mode: ", self.isContact)
        stiffness = 0.01 if self.isContact else 0
        force_error = self.d_force-self.wrench_slave.force.z
        if data.buttons[0]:
            # desired angular position
            temp.angular.x = self.curr_slave.angular.x + angxscale*data.axes[0]
            temp.angular.y = self.curr_slave.angular.y + angyscale*data.axes[1]
            temp.angular.z = self.curr_slave.angular.z + angzscale*data.axes[4]
            temp.linear.x = self.curr_slave.linear.x
            temp.linear.y = self.curr_slave.linear.y
        else:
            # desired linear position
            temp.linear.x = self.curr_slave.linear.x + \
                linxscale * data.axes[1] + stiffness*force_error*Vz[0]
            temp.linear.y = self.curr_slave.linear.y + \
                (linyscale * data.axes[0] + stiffness*force_error*Vz[1])
            temp.linear.z = self.curr_slave.linear.z + \
                stiffness * force_error*Vz[2]
            temp.angular.x = self.curr_slave.angular.x
            temp.angular.y = self.curr_slave.angular.y
            temp.angular.z = self.curr_slave.angular.z
        return temp

    def js_callback(self, js_msg):
        '''acceleration command: u = Kp*(Xm-Xs) + Kd*(dXm-dXs)'''
        self.curr_master = self.map2motion(js_msg)
        cmd = Twist()
        # linear
        cmd.linear.x = 0.0 if abs(js_msg.axes[1]) < 0.05 else \
            1.3*(self.curr_master.linear.x - self.curr_slave.linear.x) + \
            0.2*(self.d_master.linear.x - self.d_slave.linear.x)
        cmd.linear.y = 0.0 if abs(js_msg.axes[0]) < 0.05 else \
            1.4*(self.curr_master.linear.y - self.curr_slave.linear.y) + \
            0.2*(self.d_master.linear.y - self.d_slave.linear.y)
        # angular
        cmd.angular.x = 0.0 if abs(js_msg.axes[0]) < 0.05 else \
            2.8*(self.curr_master.angular.x - self.curr_slave.angular.x) + \
            0.2*(self.d_master.angular.x - self.d_slave.angular.x)
        # print(cmd.angular.x)
        cmd.angular.y = 0.0 if abs(js_msg.axes[1]) < 0.05 else \
            3.0*(self.curr_master.angular.y - self.curr_slave.angular.y) + \
            0.2*(self.d_master.angular.y - self.d_slave.angular.y)
        cmd.angular.z = 0.0 if abs(js_msg.axes[4]) < 0.05 else \
            4.2*(self.curr_master.angular.z - self.curr_slave.angular.z) + \
            0.4*(self.d_master.angular.z - self.d_slave.angular.z)
        self.cmd = cmd

    def ee_callback(self, ee_msg):
        EE_pos = ee_msg.O_T_EE_d  # inv 4x4 matrix
        self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                                EE_pos[12:16]]).transpose()

    def force_callback(self, FT_msg):
        self.wrench_slave.force.x = FT_msg.wrench.force.x
        self.wrench_slave.force.y = FT_msg.wrench.force.y
        self.wrench_slave.force.z = FT_msg.wrench.force.z

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
