#!/usr/bin/env python
# made by Jeon Bo Seong in Seoul National University
# contact junbs95@gmail.com


# required depencdecies
# {pylab, numpy, scipy, geometry_msgs, nav_msgs, rospy}
from pylab import *
import numpy as np
from scipy import ndimage
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
import rospy


# test platform node
# this node is made to mimic Platform and SLAM node
# with this code, you may find how to connect to my waypoint node with SLAM data node and Turtlebot3 node
# in this node, two objects are definded.

# (1) Platform class:
#  just gives its state in the form of geometry_msgs/Pose2D msg type
# this should be replaced with "Turtlebot3" node which basically publish its state

# (2) occupancy grid array:
# this is numpy array which contains occupancy slam information
# in this node, the data in this array is published in the form of OccupancyGrid msg type
# in actual implementation, this object should be replaced with "SLAM publishing node"



############################################################
# YOU DO NOT NEED TO READ FROM HERE! PLEASE JUMP to LINE 279
############################################################

class Platform:
    def __init__(self, state, gridmap, sintable, costable,pub_name,sub_name):
        self.state = state  # x y theta
        self.gridmap = gridmap  # x,y real position of each cell
        self.waypoint = np.array([0,0])
        # is this agent for exploration? or tracking?
        # mode is either 'explore' or 'track'
        self.sintable = sintable
        self.costable = costable
        # publisher : state
        self.pub_name = pub_name
        self.pub = rospy.Publisher(pub_name, Pose2D, queue_size=10)
        # subscribe : waypoint
        self.sub_name = sub_name
        self.sub = rospy.Subscriber(sub_name, Pose2D, self.callback)

    # publish obseraved cell (this is to be done by SLAM team)
    # Therefore below function is just to test the feasiblity of algorithm

    def publish(self):
        pose2d = Pose2D()
        pose2d.x = self.state[0]
        pose2d.y = self.state[1]
        pose2d.theta = 0  # just xy point

        self.pub.publish(pose2d)
        rospy.loginfo("published state:[ %f , %f] ", self.state[0], self.state[1])

    def callback(self, data):
        self.waypoint[0] = data.x
        self.waypoint[1] = data.y
        rospy.loginfo("received waypoint:[ %f , %f] ", self.waypoint[0], self.waypoint[1])

    def obs_data(self, occupancy_gridmap,lx,ly,dx,dy):
        # player location

        RAYS = 360  # Should be 360!

        STEP = 1  # The step of for cycle. More = Faster, but large steps may
        # cause artifacts. Step 3 is great for radius 10.

        RAD = 3.0  # FOV radius.
        N_step_RAD = 100.0

        px = self.state[0]
        py = self.state[1]

        for i in range(0, RAYS + 1, STEP):
            ax = self.sintable[i]  # Get precalculated value sin(x / (180 / pi))
            ay = self.costable[i]  # cos(x / (180 / pi))

            x = px  # Player's x
            y = py  # Player's y

            for z in range(int(N_step_RAD)):  # Cast the ray
                x += (RAD / N_step_RAD) * ax
                y += (RAD / N_step_RAD) * ay

                if x <= 0 or y <= 0 or x >= lx or y >= ly:  # If ray is out of range
                    break

                occupancy_gridmap[int(floor(x / dx))][int(floor(y / dy))] = 0  # Make tile visible

                if self.gridmap[int(floor(x / dx))][int(floor(y / dy))] == 1:  # Stop ray if it hit
                    occupancy_gridmap[int(floor(x / dx))][int(floor(y / dy))] = 1
                    break  # a wall.



sintable = [
    0.00000, 0.01745, 0.03490, 0.05234, 0.06976, 0.08716, 0.10453,
    0.12187, 0.13917, 0.15643, 0.17365, 0.19081, 0.20791, 0.22495, 0.24192,
    0.25882, 0.27564, 0.29237, 0.30902, 0.32557, 0.34202, 0.35837, 0.37461,
    0.39073, 0.40674, 0.42262, 0.43837, 0.45399, 0.46947, 0.48481, 0.50000,
    0.51504, 0.52992, 0.54464, 0.55919, 0.57358, 0.58779, 0.60182, 0.61566,
    0.62932, 0.64279, 0.65606, 0.66913, 0.68200, 0.69466, 0.70711, 0.71934,
    0.73135, 0.74314, 0.75471, 0.76604, 0.77715, 0.78801, 0.79864, 0.80902,
    0.81915, 0.82904, 0.83867, 0.84805, 0.85717, 0.86603, 0.87462, 0.88295,
    0.89101, 0.89879, 0.90631, 0.91355, 0.92050, 0.92718, 0.93358, 0.93969,
    0.94552, 0.95106, 0.95630, 0.96126, 0.96593, 0.97030, 0.97437, 0.97815,
    0.98163, 0.98481, 0.98769, 0.99027, 0.99255, 0.99452, 0.99619, 0.99756,
    0.99863, 0.99939, 0.99985, 1.00000, 0.99985, 0.99939, 0.99863, 0.99756,
    0.99619, 0.99452, 0.99255, 0.99027, 0.98769, 0.98481, 0.98163, 0.97815,
    0.97437, 0.97030, 0.96593, 0.96126, 0.95630, 0.95106, 0.94552, 0.93969,
    0.93358, 0.92718, 0.92050, 0.91355, 0.90631, 0.89879, 0.89101, 0.88295,
    0.87462, 0.86603, 0.85717, 0.84805, 0.83867, 0.82904, 0.81915, 0.80902,
    0.79864, 0.78801, 0.77715, 0.76604, 0.75471, 0.74314, 0.73135, 0.71934,
    0.70711, 0.69466, 0.68200, 0.66913, 0.65606, 0.64279, 0.62932, 0.61566,
    0.60182, 0.58779, 0.57358, 0.55919, 0.54464, 0.52992, 0.51504, 0.50000,
    0.48481, 0.46947, 0.45399, 0.43837, 0.42262, 0.40674, 0.39073, 0.37461,
    0.35837, 0.34202, 0.32557, 0.30902, 0.29237, 0.27564, 0.25882, 0.24192,
    0.22495, 0.20791, 0.19081, 0.17365, 0.15643, 0.13917, 0.12187, 0.10453,
    0.08716, 0.06976, 0.05234, 0.03490, 0.01745, 0.00000, -0.01745, -0.03490,
    -0.05234, -0.06976, -0.08716, -0.10453, -0.12187, -0.13917, -0.15643,
    -0.17365, -0.19081, -0.20791, -0.22495, -0.24192, -0.25882, -0.27564,
    -0.29237, -0.30902, -0.32557, -0.34202, -0.35837, -0.37461, -0.39073,
    -0.40674, -0.42262, -0.43837, -0.45399, -0.46947, -0.48481, -0.50000,
    -0.51504, -0.52992, -0.54464, -0.55919, -0.57358, -0.58779, -0.60182,
    -0.61566, -0.62932, -0.64279, -0.65606, -0.66913, -0.68200, -0.69466,
    -0.70711, -0.71934, -0.73135, -0.74314, -0.75471, -0.76604, -0.77715,
    -0.78801, -0.79864, -0.80902, -0.81915, -0.82904, -0.83867, -0.84805,
    -0.85717, -0.86603, -0.87462, -0.88295, -0.89101, -0.89879, -0.90631,
    -0.91355, -0.92050, -0.92718, -0.93358, -0.93969, -0.94552, -0.95106,
    -0.95630, -0.96126, -0.96593, -0.97030, -0.97437, -0.97815, -0.98163,
    -0.98481, -0.98769, -0.99027, -0.99255, -0.99452, -0.99619, -0.99756,
    -0.99863, -0.99939, -0.99985, -1.00000, -0.99985, -0.99939, -0.99863,
    -0.99756, -0.99619, -0.99452, -0.99255, -0.99027, -0.98769, -0.98481,
    -0.98163, -0.97815, -0.97437, -0.97030, -0.96593, -0.96126, -0.95630,
    -0.95106, -0.94552, -0.93969, -0.93358, -0.92718, -0.92050, -0.91355,
    -0.90631, -0.89879, -0.89101, -0.88295, -0.87462, -0.86603, -0.85717,
    -0.84805, -0.83867, -0.82904, -0.81915, -0.80902, -0.79864, -0.78801,
    -0.77715, -0.76604, -0.75471, -0.74314, -0.73135, -0.71934, -0.70711,
    -0.69466, -0.68200, -0.66913, -0.65606, -0.64279, -0.62932, -0.61566,
    -0.60182, -0.58779, -0.57358, -0.55919, -0.54464, -0.52992, -0.51504,
    -0.50000, -0.48481, -0.46947, -0.45399, -0.43837, -0.42262, -0.40674,
    -0.39073, -0.37461, -0.35837, -0.34202, -0.32557, -0.30902, -0.29237,
    -0.27564, -0.25882, -0.24192, -0.22495, -0.20791, -0.19081, -0.17365,
    -0.15643, -0.13917, -0.12187, -0.10453, -0.08716, -0.06976, -0.05234,
    -0.03490, -0.01745, -0.00000
]

costable = [
    1.00000, 0.99985, 0.99939, 0.99863, 0.99756, 0.99619, 0.99452,
    0.99255, 0.99027, 0.98769, 0.98481, 0.98163, 0.97815, 0.97437, 0.97030,
    0.96593, 0.96126, 0.95630, 0.95106, 0.94552, 0.93969, 0.93358, 0.92718,
    0.92050, 0.91355, 0.90631, 0.89879, 0.89101, 0.88295, 0.87462, 0.86603,
    0.85717, 0.84805, 0.83867, 0.82904, 0.81915, 0.80902, 0.79864, 0.78801,
    0.77715, 0.76604, 0.75471, 0.74314, 0.73135, 0.71934, 0.70711, 0.69466,
    0.68200, 0.66913, 0.65606, 0.64279, 0.62932, 0.61566, 0.60182, 0.58779,
    0.57358, 0.55919, 0.54464, 0.52992, 0.51504, 0.50000, 0.48481, 0.46947,
    0.45399, 0.43837, 0.42262, 0.40674, 0.39073, 0.37461, 0.35837, 0.34202,
    0.32557, 0.30902, 0.29237, 0.27564, 0.25882, 0.24192, 0.22495, 0.20791,
    0.19081, 0.17365, 0.15643, 0.13917, 0.12187, 0.10453, 0.08716, 0.06976,
    0.05234, 0.03490, 0.01745, 0.00000, -0.01745, -0.03490, -0.05234, -0.06976,
    -0.08716, -0.10453, -0.12187, -0.13917, -0.15643, -0.17365, -0.19081,
    -0.20791, -0.22495, -0.24192, -0.25882, -0.27564, -0.29237, -0.30902,
    -0.32557, -0.34202, -0.35837, -0.37461, -0.39073, -0.40674, -0.42262,
    -0.43837, -0.45399, -0.46947, -0.48481, -0.50000, -0.51504, -0.52992,
    -0.54464, -0.55919, -0.57358, -0.58779, -0.60182, -0.61566, -0.62932,
    -0.64279, -0.65606, -0.66913, -0.68200, -0.69466, -0.70711, -0.71934,
    -0.73135, -0.74314, -0.75471, -0.76604, -0.77715, -0.78801, -0.79864,
    -0.80902, -0.81915, -0.82904, -0.83867, -0.84805, -0.85717, -0.86603,
    -0.87462, -0.88295, -0.89101, -0.89879, -0.90631, -0.91355, -0.92050,
    -0.92718, -0.93358, -0.93969, -0.94552, -0.95106, -0.95630, -0.96126,
    -0.96593, -0.97030, -0.97437, -0.97815, -0.98163, -0.98481, -0.98769,
    -0.99027, -0.99255, -0.99452, -0.99619, -0.99756, -0.99863, -0.99939,
    -0.99985, -1.00000, -0.99985, -0.99939, -0.99863, -0.99756, -0.99619,
    -0.99452, -0.99255, -0.99027, -0.98769, -0.98481, -0.98163, -0.97815,
    -0.97437, -0.97030, -0.96593, -0.96126, -0.95630, -0.95106, -0.94552,
    -0.93969, -0.93358, -0.92718, -0.92050, -0.91355, -0.90631, -0.89879,
    -0.89101, -0.88295, -0.87462, -0.86603, -0.85717, -0.84805, -0.83867,
    -0.82904, -0.81915, -0.80902, -0.79864, -0.78801, -0.77715, -0.76604,
    -0.75471, -0.74314, -0.73135, -0.71934, -0.70711, -0.69466, -0.68200,
    -0.66913, -0.65606, -0.64279, -0.62932, -0.61566, -0.60182, -0.58779,
    -0.57358, -0.55919, -0.54464, -0.52992, -0.51504, -0.50000, -0.48481,
    -0.46947, -0.45399, -0.43837, -0.42262, -0.40674, -0.39073, -0.37461,
    -0.35837, -0.34202, -0.32557, -0.30902, -0.29237, -0.27564, -0.25882,
    -0.24192, -0.22495, -0.20791, -0.19081, -0.17365, -0.15643, -0.13917,
    -0.12187, -0.10453, -0.08716, -0.06976, -0.05234, -0.03490, -0.01745,
    -0.00000, 0.01745, 0.03490, 0.05234, 0.06976, 0.08716, 0.10453, 0.12187,
    0.13917, 0.15643, 0.17365, 0.19081, 0.20791, 0.22495, 0.24192, 0.25882,
    0.27564, 0.29237, 0.30902, 0.32557, 0.34202, 0.35837, 0.37461, 0.39073,
    0.40674, 0.42262, 0.43837, 0.45399, 0.46947, 0.48481, 0.50000, 0.51504,
    0.52992, 0.54464, 0.55919, 0.57358, 0.58779, 0.60182, 0.61566, 0.62932,
    0.64279, 0.65606, 0.66913, 0.68200, 0.69466, 0.70711, 0.71934, 0.73135,
    0.74314, 0.75471, 0.76604, 0.77715, 0.78801, 0.79864, 0.80902, 0.81915,
    0.82904, 0.83867, 0.84805, 0.85717, 0.86603, 0.87462, 0.88295, 0.89101,
    0.89879, 0.90631, 0.91355, 0.92050, 0.92718, 0.93358, 0.93969, 0.94552,
    0.95106, 0.95630, 0.96126, 0.96593, 0.97030, 0.97437, 0.97815, 0.98163,
    0.98481, 0.98769, 0.99027, 0.99255, 0.99452, 0.99619, 0.99756, 0.99863,
    0.99939, 0.99985, 1.00000
]




################################
#      map contruction
################################


lx,ly=(10,6)  # x,y scale of the map
dx,dy=(0.1,0.1)  # resolution
Nx,Ny=(int(lx/dx),int(ly/dy))
# gridmap=np.ones((Nx,Ny))*0.0

gridmap = np.ones((Nx, Ny)) * 0
# obstacle assign
for i in range(20):
    gridmap[i][40] = 1.0

for i in np.arange(40, Ny):
    gridmap[19][i] = 1.0

# obstacle assign
for i in np.arange(40, 60):
    gridmap[i][40] = 1.0

for i in arange(40, Ny):
    gridmap[40][i] = 1.0

for i in arange(40, Ny):
    gridmap[60][i] = 1.0

# obstacle assign
for i in arange(80, Nx):
    gridmap[i][40] = 1.0

for i in np.arange(40, Ny):
    gridmap[80][i] = 1.0

# obstacle assign
for i in np.arange(30, 55):
    gridmap[i][20] = 1.0

for i in arange(0, 20):
    gridmap[30][i] = 1.0

for i in arange(0, 20):
    gridmap[54][i] = 1.0

# obstacle assign
for i in np.arange(80, Nx):
    gridmap[i][10] = 1.0

for i in arange(80, Nx):
    gridmap[i][25] = 1.0

for i in arange(10, 25):
    gridmap[80][i] = 1.0




my_cmap_gridmap = matplotlib.colors.ListedColormap([(1,0,0,0),'k'])
plt.imshow(np.flipud(np.transpose(gridmap)), interpolation='none', cmap=my_cmap_gridmap, extent=[0, lx, 0, ly],
           zorder=0)
# if you want to see the map considered, uncomment the following line.
#plt.show()

####################################
#  You need to read from this line #
####################################


# from this section, you will see the "topic names" whom you should publish Turtlebot state msg(Pose2D) and occupancy map msg(OccupancyGrid)
# agent1_waypoint : topic that 1st turtlebot  should subscribe to "SNU_waypoint" node
# agent1_state : topic that 1st turtlebot should publish to "SNU_waypoint" node
#'SNU_waypoint/OccupancyGridMap' : topic you need to publish to "SNU_waypoint" node

# in this example, we assumed that two robot are in [x=4,y=2.5] and [x=3.8,y=3.8] respectively
# and the map can be displayed with uncommenting the line 276
# you can test the algorithm with changing the position provided that the position is not in obstacle




rospy.init_node('SNU_PlatformSlam_node')
rate = rospy.Rate(30)

position_agent1=np.array([4,2.5])
agent1=Platform(costable=costable,state=position_agent1,sintable=sintable,gridmap=gridmap,pub_name='agent1_state',
                sub_name='agent1_waypoint')

position_agent2=state=np.array([3.8,3.8])
agent2=Platform(costable=costable,state=np.array([3.8,3.8]),sintable=sintable,gridmap=gridmap,pub_name='agent2_state',
                sub_name='agent2_waypoint')


# the most important part!! you need to publish proper occupancy grid map
# For my algorithm, element of your gridmap array (let's say, M) M[i][j] denotes "i" th  in the direction of "x" and "j" vice versa.
# Therefore M[i][j] elements should contain occupancy information of location of the (x_origin + dx*i,y_origin + dy*j)
# Also -1 for unknown cell, 0 for open cell and 1 for occupied cell


occupancy_map=(np.ones((Nx,Ny),dtype='int')*(-1))
occ_publisher=rospy.Publisher('SNU_waypoint/OccupancyGridMap', OccupancyGrid,queue_size=10)




#######################
#      plotting      #
#######################

# make a figure + axes
fig, ax = plt.subplots(1, 1, tight_layout=True)
# make color map
my_cmap_explore = matplotlib.colors.ListedColormap([(0,0,0,0), (0,1,1,1),'k'])


# draw the boxes
# plt.imshow(np.flipud(np.transpose(occupancy_map)), cmap=my_cmap_explore, extent=[0, lx, 0, ly], zorder=0)
# plt.ion()
# plt.show()

while not rospy.is_shutdown():


    # occupancy grid map publish
    map=OccupancyGrid()
    map.data=(np.reshape(occupancy_map,(1,-1)))[0].tolist()

    #rospy.loginfo(map.data)

    #######################
    # publish the gridmap #
    #######################
    occ_publisher.publish(map)

    ####################
    # plot the gridmap #
    ####################

    # plt.imshow(np.flipud(np.transpose(occupancy_map)), interpolation='none', cmap=my_cmap_explore,
    #            extent=[0, lx, 0, ly], zorder=0)
    # plt.pause(0.05)


    #######################
    # publish agent state #
    #######################

    for agent in [agent1,agent2]:
        agent.publish()
        agent.obs_data(occupancy_map,lx,ly,dx,dy)
        #plt.plot(agent.state[0],agent.state[1],'ro')


    rate.sleep()





