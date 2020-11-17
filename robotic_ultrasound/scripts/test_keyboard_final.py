#!/usr/bin/env python
from socket import *
import sys
import csv
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import select
from geometry_msgs.msg import Twist
import keyboard
import numpy as np
from math import sin, cos, pi, atan2
import time
from franka_msgs.msg import FrankaState
import pygame
from pygame.locals import *

global read

# sending camera pose through socket
global host, port

host = "130.215.216.163"
port = 8008

read = True

def pub_vel(velcmd):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub.publish(velcmd)

def pub_pos(poscmd):
    pub = rospy.Publisher('cmd_pos', Twist, queue_size=10)
    pub.publish(poscmd)

def get_joint_pos():
    global ee_sub
    ee_sub = rospy.Subscriber("franka_state_controller/franka_states",FrankaState, callback)

def callback(data):
    global read, host, port
    EE_pos = data.O_T_EE # inv 4x4 matrix
    T_0_8 = np.array([EE_pos[0:4],EE_pos[4:8],EE_pos[8:12],EE_pos[12:16]]).transpose()
    # T_8_cam = np.array([[1.0,0.0,0.0,0.0],[0.0,0.0,-1.0,0.0],[0.0,0.0,-1.0,0.15],[0.0,0.0,0.0,1.0]])
    # T_8_cam = np.array([[0.9984,-0.0547,0.0123,-0.0248],[-0.0547,-0.9985,-0.0004,-0.0275],[0.0123,-0.0003,-0.9999,0.1570],[0.0,0.0,0.0,1.0]])
    T_8_cam = np.array([[0.999, 0.0, 0.0, 0.0],[0.0, -0.999, 0.0, -0.035],[0.0, 0.0, -0.999, 0.16],[0.0, 0.0, 0.0, 1.0]])
    T_0_cam = np.matmul(T_0_8,T_8_cam)
    ee_sub.unregister()
    msg = list()
    row = T_0_cam.shape[0]
    col = T_0_cam.shape[1]
    for i in range(0,row-1):
        for j in range(0,col):
            msg.append(T_0_cam[i][j])
    
    while read:
        '''
        with open('/home/shang/Desktop/XIHAN_script/eef_trace.csv', mode='w') as data_file:
            data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            data_writer.writerow(ee_export)
        '''
        try:
            Mysocket = socket(AF_INET,SOCK_STREAM)
            Mysocket.settimeout(2)
            Mysocket.connect((host,port))
            print("conneted ...")
            Mysocket.send(str(msg).encode())
            print("data exported")
            # print(T_0_cam)
            print(msg)
            Mysocket.close()
        except:
            print("connect failed")
            pass
        read = False


def Goal_corr(Radius_desired,pitch,yaw,x_obj,y_obj,z_obj,goal_x,goal_y,goal_z):
    R = 0.738
    # R = 0.4
    Radius = Radius_desired
    radius_jump = 0.0005

    # pose = angle2quat #convert angles to quaternions
    '''
    if np.sqrt(x_obj*x_obj + y_obj*y_obj) > R:
        print('Get closer please')
    '''
    if np.sqrt(goal_x*goal_x + goal_y*goal_y) > R:
        Radius_desired = R - np.sqrt(x_obj*x_obj+y_obj*y_obj) # radius correction
        # Radius = R - np.sqrt(x_obj*x_obj+y_obj*y_obj) # radius correction

    if Radius >= Radius_desired:
        Radius -= radius_jump       # ramping down radius
        # print("Fixing Radius to: %f\n" % Radius)
    else:
        Radius = Radius
    
    goal_x = x_obj - Radius*cos(pitch)*cos(yaw)
    goal_y = y_obj - Radius*cos(pitch)*sin(yaw)

    if goal_x <= 0.25:
        goal_x = 0.25    # prevent hitting base
    if goal_z <= 0.18:
        goal_z = 0.18    # prevent hitting table
    print("R: %f, x: %f, y: %f, z: %f pitch: %f yaw: %f\n" %(Radius,goal_x,goal_y,goal_z,pitch,yaw))

    return goal_x, goal_y, goal_z

def calcGoal(Radius,pitch,yaw,x_obj,y_obj,z_obj):

    goal_x = x_obj - cos(yaw)*cos(pitch)*Radius
    goal_y = y_obj - sin(yaw)*cos(pitch)*Radius
    goal_z = z_obj + sin(pitch)*Radius
    
    [goal_x,goal_y,goal_z] = Goal_corr(Radius,pitch,yaw,x_obj,y_obj,z_obj,goal_x,goal_y,goal_z) 
    
    return goal_x, goal_y, goal_z

def main():
    global read, host, port
    rospy.init_node('FrankaKeyTeleop', anonymous=True)

    ## setting home pose
    init_point = [0.4,0,0.35]  
    # global T_0_cam

    # pose_goal = geometry_msgs.msg.Pose()
    goal_x = init_point[0]
    goal_y = init_point[1]
    goal_z = init_point[2]+0.115
    pre_ts = time.time()
    pre_pos = [0,0,0,0,0,0]

    # position of object
    x_obj = 0.65
    y_obj = 0.0
    z_obj = 0.16

    cmd_pos = Twist()
    cmd_vel = Twist()

    yaw_step = 0.00006
    radius_step = 0.00005
    pitch_step = 0.00002
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    Radius = 0.0
    max_Radius = 0.5
    max_yaw = 1.50   
    max_pitch = 1.0
    min_pitch = -0.2

    # eef_pose = list()       # for data recording

    # init_time = time.time()
    pygame.init()
    screen = pygame.display.set_mode((560,400))
    insimg = pygame.image.load('RCM_key_inst.png')
    pygame.display.set_caption('Franka Emika Panda Keyboard Teleop Console')
    screen.blit(insimg,(0,0))
    pygame.display.update()

    print('System initialization succeed')
    print('Moving to Initial Position')

    while True:
        rt_time = time.time()
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        
        if keys[K_q]:

            end_msg = 'q'
            try:
                Mysocket = socket(AF_INET,SOCK_STREAM)
                Mysocket.settimeout(1)
                Mysocket.connect((host,port))
                print("quit ...")
                Mysocket.send(str(end_msg).encode())
                Mysocket.close()
            except:
                print("connect failed")
                pass
            pygame.quit()
            break

        if keys[K_w]:

            Radius += radius_step
            print("current radius: %f" % Radius)
            if Radius >= max_Radius:
                Radius = max_Radius

        elif keys[K_s]:

            Radius -= radius_step
            print("current radius: %f" % Radius)
            if Radius <= 0:
                Radius = 0

        elif keys[K_o]:

            goal_x = init_point[0]
            goal_y = init_point[1]
            goal_z = init_point[2]+0.115
            pitch = 0
            yaw = 0
            print("go back to origin")

        elif keys[K_UP]:

            pitch += pitch_step
            if pitch > max_pitch:
                pitch = max_pitch
            [goal_x,goal_y,goal_z] = calcGoal(Radius,pitch,yaw,x_obj,y_obj,z_obj)

        elif keys[K_DOWN]:

            pitch -= pitch_step
            if pitch < min_pitch:
                pitch = min_pitch
            [goal_x,goal_y,goal_z] = calcGoal(Radius,pitch,yaw,x_obj,y_obj,z_obj)

        elif keys[K_LEFT]:

            yaw -= yaw_step
            if yaw < -max_yaw:
                yaw = -max_yaw
            [goal_x,goal_y,goal_z] = calcGoal(Radius,pitch,yaw,x_obj,y_obj,z_obj)

        elif keys[K_RIGHT]:

            yaw += yaw_step
            if yaw > max_yaw:
                yaw = max_yaw
            [goal_x,goal_y,goal_z] = calcGoal(Radius,pitch,yaw,x_obj,y_obj,z_obj)

        elif keys[K_r]:

            if read:
                get_joint_pos()
            else:
                read = True
            time.sleep(1.0)
        
        else:
            goal_x += 0 #-0.00005 + 0.0001*np.random.rand()
            goal_y += 0
            goal_z += 0
        
        
        if roll > pi:
            roll -= 2*pi
        if roll < -pi:
            roll += 2*pi
        if pitch > pi:
            pitch -= 2*pi
        if pitch < -pi:
            pitch += 2*pi
        if yaw > pi:
            yaw -= 2*pi
        if yaw < -pi:
            yaw += 2*pi

        # data = [x, y, z, roll, pitch, yaw]

        cmd_pos.linear.x = goal_x
        cmd_pos.linear.y = goal_y
        cmd_pos.linear.z = goal_z
        cmd_pos.angular.x = roll
        cmd_pos.angular.y = pitch
        cmd_pos.angular.z = yaw

        # Desired Vel
        cmd_vel.linear.x = (goal_x-pre_pos[0])/(rt_time-pre_ts)
        cmd_vel.linear.y = (goal_y-pre_pos[1])/(rt_time-pre_ts)
        cmd_vel.linear.z = (goal_z-pre_pos[2])/(rt_time-pre_ts)
        cmd_vel.angular.x = (roll-pre_pos[3])/(rt_time-pre_ts)
        cmd_vel.angular.y = (pitch-pre_pos[4])/(rt_time-pre_ts)
        cmd_vel.angular.z = (yaw-pre_pos[5])/(rt_time-pre_ts)

        pub_pos(cmd_pos)
        pub_vel(cmd_vel)

        # if yaw < -1.5 or yaw > 1.5:
        #     print("x: %f, y: %f, z: %f pitch: %f yaw: %f\n" %(cmd_pos.linear.x,cmd_pos.linear.y,cmd_pos.linear.z, cmd_pos.angular.y, cmd_pos.angular.z))

        pre_ts = rt_time
        pre_pos = [goal_x, goal_y, goal_z, roll, pitch, yaw]
        # time.sleep(0.0001)

    pygame.quit()
    print('System shutdown success')
    return 0


if __name__ == '__main__':
    main()

