#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from sys import path
import os

import rospy
import array

from std_msgs.msg import UInt16
from std_msgs.msg import String
from bodyhub.msg import JointControlPoint
from bodyhub.srv import *
# from actexecpackage.srv import *


class BodyState(object):
    def __init__(self):
        self.control_id = 1
        self.srv_jump = '/MediumSize/BodyHub/StateJump'

    def get_id(self):
        return self.control_id

    def set_state(self, state):
        rospy.wait_for_service(self.srv_jump)
        bodyhub_client = rospy.ServiceProxy(self.srv_jump, SrvState)
        bodyhub_client(self.control_id, state)

    def get_state(self):
        return None

    def acquire(self):
        self.set_state('setStatus')

    def release(self):
        self.set_state('break')
        self.set_state('reset')
