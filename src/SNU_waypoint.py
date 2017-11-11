#!/usr/bin/env python

# required depencdecies
# {pylab, numpy, scipy, geometry_msgs, nav_msgs, rospy}

import matplotlib.pyplot as plt
import matplotlib
from pylab import *
import numpy as np
from scipy import ndimage
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
import rospy



class Agent:
    def __init__(self,pub_name,sub_name):
        # initial state
        self.state = [999,999]  # x y theta

        self.waypoint = None

        # publisher : waypoint
        self.pub_name = pub_name
        self.pub = rospy.Publisher(pub_name, Pose2D, queue_size=10)
        # subscriber : state
        self.sub_name = sub_name
        self.sub = rospy.Subscriber(sub_name, Pose2D, self.callback)
        # agent is not assgined way point from master class
        self.waypoint_init_flag = False
        # callback the state information from platform

    def callback(self, data):
        self.state[0] = data.x
        self.state[1] = data.y

    # publish waypoint
    def publish(self):
        pose2d = Pose2D()
        pose2d.x = self.waypoint[0]
        pose2d.y = self.waypoint[1]
        pose2d.theta = 0  # just xy point

        if self.waypoint_init_flag:
            self.pub.publish(pose2d)
            rospy.loginfo("published waypoint:[ %f , %f] ", self.waypoint[0], self.waypoint[1])
        else:
            rospy.loginfo("don't have waypoint to publish yet.")


class Master:
    def __init__(self, agent_list, dx, dy,Nx,Ny,origin):

        # occupancy grid
        self.occupancy_grid = np.ones((Nx, Ny)) * (-1)  # initially all cell is unknown
        # frontier map : same size with occupancy grid but contains frontier cells
        self.frontier_map = np.zeros((Nx, Ny))

        self.agent_list = agent_list
        # centroid of clustered frontiers
        self.cent_xs = None
        self.cent_ys = None

        # grid size = resolution

        self.dx = dx
        self.dy = dy

        # if there are no frontier, then search is finished
        self.search_fin = False

        # origin of gridmap
        self.origin = origin

        # subscribe occupancy map

        self.sub = rospy.Subscriber('SNU_waypoint/OccupancyGridMap', OccupancyGrid, self.callback)
        self.Nx=Nx
        self.Ny=Ny

        self.occumap_recieved_flag=False

    # occupancy callback

    def callback(self, data):
        self.occupancy_grid = np.reshape(np.array(data.data),(self.Nx,self.Ny))
        self.occumap_recieved_flag=True



    # this function gives each agent the waypoint to be followed

    def waypoint(self):


        if not self.search_fin:  # if not search finished

            # number of cluster of frontier=1 CAUTION: TWO AGENT CAN COLLIDE WITH LITTLE POSSIBLITY !!
            if len(self.cent_xs) == 1:

                frontier_idx = zip(*np.where(self.frontier_map == 1))

                N_frontier = len(frontier_idx)

                for agent in self.agent_list:

                    dists = [np.power((self.origin[0] + i * self.dx - agent.state[0]) ** 2 + (
                    self.origin[0] + j * self.dy - agent.state[1]) ** 2, 0.5)
                             for i, j in frontier_idx]
                    sorted_idx = sorted(range(len(dists)), key=lambda k: dists[k])
                    N_nearest_idx = np.array(sorted_idx[1:N_frontier / 2], dtype='int')

                    cent_x = self.origin[0]
                    cent_y = self.origin[1]
                    count = 0.0
                    nearest_frontier_idx = [frontier_idx[i] for i in N_nearest_idx]

                    for i, j in nearest_frontier_idx:
                        count += 1
                        cent_x += i * self.dx
                        cent_y += j * self.dy

                    cent_x = cent_x / count
                    cent_y = cent_y / count



                    agent.waypoint = np.array([cent_x, cent_y])
                    agent.waypoint_init_flag=True

            else:  # number of cluster of frontier>1
                cent_idx = range(len(self.cent_xs))  # number of centroids
                dists_arr = []
                for agent in self.agent_list:  # 2 number assumed
                    dists = [
                        np.power((self.cent_xs[i] - agent.state[0]) ** 2 + (self.cent_ys[i] - agent.state[1]) ** 2, 0.5)
                        for i in cent_idx]
                    dists_arr.append(dists)

                min_cost = 9999
                min_i, min_j = (0, 0)

                for i in cent_idx:  # agent1
                    for j in cent_idx:  # agent2
                        if not i == j:
                            cost = dists_arr[0][i] + dists_arr[1][j]
                            if cost < min_cost:
                                min_cost = cost
                                min_i, min_j = (i, j)



                self.agent_list[0].waypoint = np.array([self.cent_xs[min_i], self.cent_ys[min_i]])
                self.agent_list[0].waypoint_init_flag=True
                self.agent_list[1].waypoint = np.array([self.cent_xs[min_j], self.cent_ys[min_j]])
                self.agent_list[1].waypoint_init_flag=True
        else:
            print 'search_finished!'   # because all cells are explored

    def get_frontier(self):

        ############
        # frontier #
        ############

        # cell value of occupancy grid (nav_msgs/OccupancyGrid.msg)

        # open cell: value=-1
        # occupied cell:value =1
        # unoccupied cell: value=0


        # get frontier: open cells which have unoccupied cell as its neighbor


        # (1) frontier detection

        indices = zip(*np.where(self.occupancy_grid == -1))  # for all open cells

        frontier_x = []
        frontier_y = []

        for i, j in indices:
            # 4 point connectivity
            if not i < 1:
                if self.occupancy_grid[i - 1][j] ==0:
                    frontier_x.append(i)
                    frontier_y.append(j)

            if not i > self.Nx - 2:
                if self.occupancy_grid[i + 1][j] ==0:
                    frontier_x.append(i)
                    frontier_y.append(j)

            if not j < 1:
                if self.occupancy_grid[i][j - 1] == 0:
                    frontier_x.append(i)
                    frontier_y.append(j)

            if not j > self.Ny - 2:
                if self.occupancy_grid[i][j + 1] == 0:
                    frontier_x.append(i)
                    frontier_y.append(j)

        frontier_map = np.zeros((self.Nx,self. Ny))
        for i, j in zip(frontier_x, frontier_y):
            frontier_map[i][j] = 1

        # (2) frontier clustering

        frontier_map_clusterd, num_feat = ndimage.measurements.label(frontier_map, np.ones((3, 3)))

        self.frontier_map = frontier_map

        # (3) get frontier centriods for each cluster

        cent_xs = []
        cent_ys = []

        for cluster_idx in range(num_feat):
            this_cluster_idx = zip(*np.where(frontier_map_clusterd == cluster_idx + 1))


            ##################################################################################################################
            # ignore_num = frontier cluster that has too small number of frontier cells => will not be conceived as cluster  #
            ##################################################################################################################

            ignore_num=4
            if len(this_cluster_idx) > ignore_num:
                cent_x = self.origin[0]
                cent_y = self.origin[1]
                count = 0.0

                for i, j in this_cluster_idx:
                    count += 1
                    cent_x += i * self.dx
                    cent_y += j * self.dy

                cent_x = cent_x / count
                cent_y = cent_y / count

                cent_xs.append(cent_x)
                cent_ys.append(cent_y)

        self.cent_xs = cent_xs
        self.cent_ys = cent_ys

        if (len(self.cent_xs) == 0):
            self.search_fin = True
            print 'search finished!'


if __name__ == '__main__':


    rospy.init_node('SNU_waypoint_node')
    rate = rospy.Rate(30)

    # we need to know the followings

    # (1) map size (Nx,Ny)
    # (2) resolution (dx,dy) of Occupancy map
    # (3) origin of map (w.r.t inertial coord)
    ##############################



    ################################
    # Please fill this information #
    ################################


    # map information
    lx, ly = (10, 6)  # x,y scale of the map
    dx, dy = (0.1, 0.1)  # resolution
    Nx, Ny = (int(lx / dx), int(ly / dy))   # shape of occupancy grid map
    origin=np.array([0,0])
    ##############################

    agent1=Agent(pub_name='agent1_waypoint',sub_name='agent1_state')
    agent2=Agent(pub_name='agent2_waypoint', sub_name='agent2_state')

    agent_list=[agent1,agent2]

    master=Master(agent_list=agent_list,dx=dx,dy=dy,Nx=Nx,Ny=Ny,origin=origin)


    # for plotting
    fig, ax = plt.subplots(1, 1, tight_layout=True)

    # make color map
    my_cmap_explore = matplotlib.colors.ListedColormap([(0, 0, 0, 0), (0, 1, 1, 1), 'k'])
    my_cmap_frontier = matplotlib.colors.ListedColormap([(1, 0, 0, 0), 'y'])

    ###############################################
    # if you want to close the plot, then comment #
    ###############################################

    plt.imshow(np.flipud(np.transpose(master.frontier_map)),interpolation=None, cmap=my_cmap_frontier, extent=[0, lx, 0, ly], zorder=0)
    plt.ion()
    plt.show()


    while not rospy.is_shutdown():
        # get froniter cell from occupancy gridmap


        # looping only if we are connected to node that publishes occupancymap
        isok = master.sub.get_num_connections() > 0

        if isok & master.occumap_recieved_flag:
            master.get_frontier()
            # each agent in master updates waypoint

            master.waypoint()
            # waypoint publish
            for agent in master.agent_list:
                agent.publish()


            ########################################################
            # if you see what's happening, just leave this section #
            ########################################################

            # plot frontier map
            plt.imshow(np.flipud(np.transpose(master.frontier_map)),interpolation=None, cmap=my_cmap_frontier, extent=[0, lx, 0, ly], zorder=0)
            # plot recieved occupancy map
            plt.imshow(np.flipud(np.transpose(master.occupancy_grid)),interpolation=None, cmap=my_cmap_explore, extent=[0, lx, 0, ly], zorder=0)

            # plot agent and their waypoint
            plt.plot(master.agent_list[0].state[0], master.agent_list[0].state[1], 'ro')
            plt.plot(master.agent_list[1].state[0], master.agent_list[1].state[1], 'bo')

            plt.plot(master.agent_list[0].waypoint[0], master.agent_list[0].waypoint[1], 'r*')
            plt.plot(master.agent_list[1].waypoint[0], master.agent_list[1].waypoint[1], 'b*')

            plt.pause(0.01)
            fig.clear()

        else:
            print 'does not connected to platform'



        rate.sleep()









