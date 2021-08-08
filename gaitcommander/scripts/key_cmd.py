#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import time
import tty
import termios
import select

import rospy
import rospkg

from std_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli

STEP_LEN = [0.1, 0.05, 10.0]
STEP_NUM = 6

class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        rospy.init_node(name, anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def start(self):
        print('action start')


class Walking(Action):
    def __init__(self):
        super(Walking, self).__init__('walk_telecontrol', 2)
        self.step_len = STEP_LEN
        self.step_num = STEP_NUM
        self.timeout = 15
        self.gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)

    def printTeleInfo(self):
        print('\n%-15s%s' % (' ', 'w--forward'))
        print('%-15s%-15s%-15s' % ('a--left', 's--backward', 'd--right'))
        print('%-15s%-15s%-15s' % ('z--trun left', 'x--in situ', 'c--trun right'))
        print('%-15s%s\n' % (' ', 'q--quit'))

    def walking(self, delta_x, delta_y, theta):
        rospy.wait_for_message('/requestGaitCommand', Bool, self.timeout)
        self.gait_cmd_pub.publish(data=[delta_x, delta_y, theta])

    def walking_n_steps(self, x, y, a, n):
        for i in range(0, n):
            self.walking(x, y, a)
            print('%s %-10s%-10s%-10s%-10s'%('step', i+1, x, y, a))
        self.printTeleInfo()

    def getch(self, str=''):
        print(str,)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print(ch)
        return ch

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyboard_poll(self):
        while not rospy.is_shutdown():
            cmd = self.getch('key:')
            if cmd == 'w':
                self.walking_n_steps(self.step_len[0], 0.0, 0.0, self.step_num)
            elif cmd == 's':
                self.walking_n_steps(-self.step_len[0]*0.8, 0.0, 0.0, self.step_num)
            elif cmd == 'a':
                self.walking_n_steps(0, self.step_len[1], 0.0, self.step_num)
            elif cmd == 'd':
                self.walking_n_steps(0, -self.step_len[1], 0.0, self.step_num)
            elif cmd == 'z':
                self.walking_n_steps(0.0, 0.0, self.step_len[2], self.step_num)
            elif cmd == 'c':
                self.walking_n_steps(0.0, 0.0, -self.step_len[2], self.step_num)
            elif cmd == 'x':
                self.walking_n_steps(0.0, 0.0, 0.0, self.step_num)
            elif cmd == 'q':
                return

    def start(self):
        self.bodyhub_walk()
        self.printTeleInfo()
        self.keyboard_poll()
        rospy.signal_shutdown('exit')


if __name__ == '__main__':
    Walking().start()
