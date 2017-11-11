#!/usr/bin/env python


import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib
from pylab import *
import matplotlib.patches as patches
import itertools
import rospy
from geometry_msgs.msg import Pose2D


class Platform:
    def __init__(self,state,pub_name,sub_name):
        self. state =state #initial x y theta

        # setpoint
        self.setpoint =np.array([0,0,0])    # it is Pose2D object
        self.pub_name=pub_name #it publish its state
        self.sub_name=sub_name #it subscribe waypoint

        # we should match each topic with each platform
        self.pub=rospy.Publisher(pub_name,Pose2D,queue_size=10)
        self.sub=rospy.Subscriber(sub_name,Pose2D,self.callback)
        self.setpoint_init_flag=False

    def callback(self,data):
        # subscribe setpoint
        self.setpoint_init_flag=True
        self.setpoint[0]=data.x
        self.setpoint[1]=data.y
        self.setpoint[2]=data.theta


    def publish(self):
        pose2d=Pose2D()
        pose2d.x=self.state[0]
        pose2d.y =self.state[1]
        pose2d.theta = self.state[2]

        self.pub.publish(pose2d)


    def following(self, Kx, Ktheta, dt, verbose):
        if not self.setpoint_init_flag==False:
            xt = self.setpoint[0] # target pos x
            yt = self.setpoint[1] # target pos y
            theta = self.state[2]

            theta_target = np.arctan2(yt - self.state[1], xt - self.state[0])
            if theta_target < 0:
                theta_target += 2 * np.pi

            if (theta_target - theta) > np.pi:
                cmd_theta = theta_target - theta - 2 * np.pi
            else:
                cmd_theta = theta_target - theta

            if verbose:
                print "cmd theta :{} / theta :{} / theta_target: {}".format(cmd_theta * 180.0 / np.pi,
                                                                            theta * 180.0 / np.pi,
                                                                            theta_target * 180.0 / np.pi)

            cmd = np.array((Kx * (xt - self.state[0]),
                            Kx * (yt - self.state[1]),
                            Ktheta * (cmd_theta)))

            self.state += cmd * dt


if __name__=="__main__":
    ########################
    # Spawn platform agent #
    ########################

    rospy.init_node('plaform')
    rate = rospy.Rate(10)

    platform1 = Platform([0, 1, pi / 3],pub_name='agent1_state',sub_name='agent1_setpoint')
    platform2 = Platform([5, 15, pi / 3 + pi],pub_name='agent2_state',sub_name='agent2_setpoint')


    while not rospy.is_shutdown():
        #state publish
        platform1.publish()
        platform2.publish()

        platform1.following(Kx=0.2,Ktheta=3,dt=0.2,verbose=False)
        platform2.following(Kx=0.2,Ktheta=3,dt=0.2,verbose=False)


        rate.sleep()



