#!/usr/bin/env python

###################
## 1.Initialize  ##
###################

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib
from pylab import *
import numpy as np
import matplotlib.patches as patches
import itertools
import SNU_header
from SNU_header import *

rospy.init_node('SNU_waypoint_node')

fig = plt.figure()
ax = fig.add_subplot(111)

###############
# Map setting #
###############

ws_range = np.array([[0.0, 10.0], [0.0, 20.0]])  # [xrange , yrange]
Lx = ws_range[0][1] - ws_range[0][0]
Ly = ws_range[1][1] - ws_range[1][0]
Nx, Ny = (20, 40)
dx, dy = (Lx / Nx, Ly / Ny)
# target_idx=150;

pos_map = [np.array((ws_range[0][0] + dx / 2 + dx * (x), ws_range[1][0] + dy / 2 + dy * (y))) for x in range(Nx) for y
           in range(Ny)]
prob_map = Prob_map(pos_map, dx, dy,Nx,Ny)

################
# Agent spawn  #
################
agent1 = Agent([0, 1, pi / 3], pos_map, [2, pi / 2], target_idx=None, mode='explore',
               pub_name='agent1_setpoint',sub_name='agent1_state')
agent1.draw_agent()

agent2 = Agent([5, 15, pi / 3 + pi], pos_map, [2, pi / 2], target_idx=None, mode='explore',
               pub_name='agent2_setpoint',sub_name='agent2_state')
agent2.draw_agent()

agent_list = [agent1, agent2]

################
# Master spawn #
################

master = Master(pos_map, prob_map, agent_list)
master.prob_map_update()
for agent in master.agent_list:
    master.waypoint(agent)

    agent.publish()
    plt.scatter(pos_map[agent.setpoint][0], pos_map[agent.setpoint][1], c='r', marker='*')

# ------ plotting----------------------
plt.ion()
plt.axis('equal')

master.prob_map.plot_map(ax)
# colorbar()
# tgt_pos=pos_map[target_idx]
# plt.scatter(tgt_pos[0],tgt_pos[1],c='r',marker='*')
plt.axis([ws_range[0][0], ws_range[0][1], ws_range[1][0], ws_range[1][1]])
plt.show()
# ------ plotting----------------------



rate=rospy.Rate(30)

###################
## 2.Main loop   ##
###################
fig.clear()
#ros loop
while not rospy.is_shutdown():
    # for agent in master.agent_list:
    #     # control part is not our role. this is run in other platform
    #     # actually, we need to receive positional info from platform node
    #     # agent.following(Kx=0.2, Ktheta=3, dt=0.2, verbose=False)


    master.prob_map_update()

    for agent in master.agent_list:
        plt.scatter(pos_map[agent.setpoint][0], pos_map[agent.setpoint][1], c='r', marker='*')
        agent.draw_agent()
        if not agent.setpoint==None:
            agent.publish()

        if agent.isreached():
            print 'way point changed!'
            master.waypoint(agent)

            # --------- agent drawing---------------------
            plt.scatter(pos_map[agent.setpoint][0], pos_map[agent.setpoint][1], c='r', marker='*')

    # -------------------- plotting --------------------------
    plt.axis('equal')
    plt.axis([ws_range[0][0], ws_range[0][1], ws_range[1][0], ws_range[1][1]])
    master.prob_map.plot_map(ax=ax)
    plt.pause(0.01)
    fig.clear()

    rate.sleep()







