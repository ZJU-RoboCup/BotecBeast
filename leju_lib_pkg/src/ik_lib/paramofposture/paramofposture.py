#!/usr/bin/env python
# coding=utf-8
import math
import numpy as np
# import eigen as e

class Pose(object):
    def __init__(self):
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class ParameterOfPosture(object):
    def __init__(self):
        '''
        定义右手系，机器人前方为x轴正方形，左方为y轴正方向，上方为z轴正方向
        '''
        self.Lfoot_x = 0.0
        self.Lfoot_y = 0.0
        self.Lfoot_z = 0.0
        self.Lfoot_R = 0.0
        self.Lfoot_P = 0.0
        self.Lfoot_Y = 0.0

        self.Rfoot_x = 0.0
        self.Rfoot_y = 0.0
        self.Rfoot_z = 0.0
        self.Rfoot_R = 0.0
        self.Rfoot_P = 0.0
        self.Rfoot_Y = 0.0

        self.Torso_x = 0.0
        self.Torso_y = 0.0
        self.Torso_z = 0.0
        self.Torso_R = 0.0
        self.Torso_P = 0.0
        self.Torso_Y = 0.0

        self.LArm_x = 0.0
        self.LArm_y = 0.0
        self.LArm_z = 0.0
        
        self.RArm_x = 0.0
        self.RArm_y = 0.0
        self.RArm_z = 0.0


    def getleftlegpos(self):
        torso_eulor = np.array([-self.Torso_P,-self.Torso_Y,-self.Torso_R])
        torso_rotation = self.euler2rotation(torso_eulor)

        left_eulor = np.array([self.Lfoot_P-self.Torso_P,self.Lfoot_Y-self.Torso_Y,self.Lfoot_R-self.Torso_R])
        left_rotation = self.euler2rotation(left_eulor)

        left_vector_wf = np.array([(self.Lfoot_x-self.Torso_x), (self.Lfoot_y-self.Torso_y), (self.Lfoot_z-self.Torso_z)])
        left_vector_wf = np.matmul(torso_rotation, left_vector_wf)
        #######################################################
        quat = self.rot2quaterninous(left_rotation.T)

        left_pose = Pose()
        left_pose.X = left_vector_wf[0]
        left_pose.Y = left_vector_wf[1]
        left_pose.Z = left_vector_wf[2]
        left_pose.w = quat['w']
        left_pose.x = quat['x']
        left_pose.y = quat['y']
        left_pose.z = quat['z']
        return left_pose

    def getrightlegpos(self):

        torso_eulor = np.array([-self.Torso_P,-self.Torso_Y,-self.Torso_R])
        torso_rotation = self.euler2rotation(torso_eulor)

        right_eulor = np.array([self.Rfoot_P-self.Torso_P,self.Rfoot_Y-self.Torso_Y,self.Rfoot_R-self.Torso_R])
        right_rotation = self.euler2rotation(right_eulor)

        right_vector_wf = np.array([(self.Rfoot_x-self.Torso_x), (self.Rfoot_y-self.Torso_y), (self.Rfoot_z-self.Torso_z)])
        right_vector_wf = np.matmul(torso_rotation, right_vector_wf)
        #######################################################
        quat = self.rot2quaterninous(right_rotation.T)

        right_pose = Pose()
        right_pose.X = right_vector_wf[0]
        right_pose.Y = right_vector_wf[1]
        right_pose.Z = right_vector_wf[2]
        right_pose.w = quat['w']
        right_pose.x = quat['x']
        right_pose.y = quat['y']
        right_pose.z = quat['z']
        # print(right_vector_wf)
        return right_pose

    def euler2rotation(self, euler):
        heading = euler[0]
        attitude = euler[1]
        bank = euler[2]
        ch = math.cos(heading)
        sh = math.sin(heading)
        ca = math.cos(attitude)
        sa = math.sin(attitude)
        cb = math.cos(bank)
        sb = math.sin(bank)

        rot = np.zeros((3,3),dtype=float)
        rot[0, 0] = ch * ca
        rot[0, 1] = sh*sb - ch*sa*cb
        rot[0, 2] = ch*sa*sb + sh*cb
        rot[1, 0] = sa
        rot[1, 1] = ca*cb
        rot[1, 2] = -ca*sb
        rot[2, 0] = -sh*ca
        rot[2, 1] = sh*sa*cb + ch*sb
        rot[2, 2] = -sh*sa*sb + ch*cb
        return rot

    def rot2quaterninous(self,rotation):
        trace = np.trace(rotation)
        w = math.sqrt(trace+1)/2.0
        x = (rotation[2,1]-rotation[1,2])/(4*w)
        y = (rotation[0,2]-rotation[2,0])/(4*w)
        z = (rotation[1,0]-rotation[0,1])/(4*w)
        q = {'w':w, 'x':x, 'y':y, 'z':z}
        return q

    # def test(self):
    #     quat = self.rot2quaterninous(self.euler2rotation([-0.00236031,0,-0.000331719]))
    #     rot = e.Quaterniond(quat['w'],quat['x'],quat['y'],quat['z']).matrix()
    #     print(rot)
    #     # print(self.euler2rotation([-0.00236031, 0.0, -0.000331719]))


if __name__ == '__main__':
    print("Parameter Of torso posture ")
