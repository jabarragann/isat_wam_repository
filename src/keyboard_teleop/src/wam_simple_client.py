#!/usr/bin/env python

"""
Description: 
    the program which governs communication with WAM robot
    get real-time heartbeat from WAM to know its position
    and send target joint position for WAM to go to

Sample usage:
    wam = WAM()
    wam.init_socket(host='128.46.125.212', port=4000, buflen=256)
    wam.query_joint_position()
    wam.move_joint([0, 0, 0, 0, 0, 0, 0])
    wam.go_home()

Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Nov 2, 2017

License: 
    GNU General Public License v3
"""

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
import socket
import numpy as np


def pprint(name, l):
    l_str = ''.join(['%.3f ' % item for item in l])
    print name + ': ' + l_str

class WAM:
    def __init__(self):
        # init node for wam_node
        rospy.init_node('wam_node')
        
    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting moveWAM_BRIDGE()...")
        if self.socket:
            self.go_home()
            self.socket.close()
            print "Socket closed properly..."

    def init_socket(self, host, port, buflen):
        # init socket with Multimodal.exe in Windows C++
        if host == 'local_file':
            file = open("/home/santi/ros_workspaces/lunar_ws/src/keyboard_teleop/src/WAM_IP.txt", "r") 
            host = file.read()
            print "recovered WAM IP %s from local file..." % host

        # create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buflen = buflen

        # connection to hostname on the port.
        re_try = 1
        while re_try and not rospy.is_shutdown():
            try:
                self.socket.connect((host, port))
                re_try = 0
            except:
                print("Connection failed. Retry after 0.1 seconds")
                re_try = 1
                time.sleep(0.1)
                continue

        print "Built socket connection with WAM PC..."
        print "Heartbeat started to get real-time robot joint positions..."
        print "Wait for path planning result to move the robot..."
    
    def query_joint_pose(self):
        self.socket.send('2')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def query_cart_pose(self):
        self.socket.send('9')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def goto_joint(self, targetJp):
        # publish the command to go to joint
        assert (len(targetJp) == 7)
        msg = '7 ' + ''.join([str(i)+' ' for i in targetJp])
        self.socket.send(msg)
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        assert (pac == 'complete')
        return pac 

    def goto_XYZ(self, targetCp, fixQuat):
        assert (len(targetCp) == 3)
        
        msg = '8 ' + ''.join([str(i)+' ' for i in targetCp])
        msg += '%i ' % fixQuat
        self.socket.send(msg)
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        assert (pac == 'complete')
        return pac 

    def go_home(self):
        print "go home WAM you are drunk!!!"
        self.socket.send('4')
        time.sleep(1)
            
    def decode_joint_pac(self, pac):
        pac_split = pac.split(' ')
        joints = []
        for i, item in enumerate(pac_split):
            if i == 6:
                item = item[:8]
            joint_float = float(item)
            joints.append(joint_float)

        joints_dict = {}
        joints_dict['j1'] = joints[0]
        joints_dict['j2'] = joints[1]
        joints_dict['j3'] = joints[2]
        joints_dict['j4'] = joints[3]
        joints_dict['j5'] = joints[4]
        joints_dict['j6'] = joints[5]
        joints_dict['j7'] = joints[6]

        return joints_dict

    def decode_pose_pac(self, pac):
        pac_split = pac.split(' ')
        pose = []
        for i, item in enumerate(pac_split):
            if i == 0:
                continue

            if i == 7:
                item = item[:8]
            pos_float = float(item)
            pose.append(pos_float)

        pos_dict = {}
        pos_dict['x'] = pose[0]
        pos_dict['y'] = pose[1]
        pos_dict['z'] = pose[2]
        pos_dict['q_w'] = pose[3]
        pos_dict['q_x'] = pose[4]
        pos_dict['q_y'] = pose[5]
        pos_dict['q_z'] = pose[6]
        return pos_dict

    def run(self):
        rospy.on_shutdown(self.clean_shutdown)
        while not rospy.is_shutdown():
            pac = self.query_joint_pose()
            joints_dict = self.decode_joint_pac(pac)
            print("current joint pose: ", joints_dict)

            pac  = self.query_cart_pose()
            pose_dict = self.decode_pose_pac(pac)
            print("current cart pose: ", pose_dict)
            
        rospy.signal_shutdown("run() finished...")

if __name__ == '__main__':
    try:
        wam = WAM()
        wam.init_socket(host='local_file', port=4000, buflen=256)
        wam.run()
    except KeyboardInterrupt:
        print("Ok ok, keyboard interrupt, quitting")
        sys.exit(1)
    else:
        print("Normal termination")
        sys.exit(2)