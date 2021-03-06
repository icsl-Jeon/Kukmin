from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import matplotlib
from pylab import *
import numpy as np
import matplotlib.patches as patches
import itertools
import rospy
from geometry_msgs.msg import Pose2D


# this is agent class definition
# agent class communicates with real platform(turtlebot)
# and is included also to master


class Agent:

    def __init__(self ,state ,pos_map ,sensor_spec ,target_idx ,mode,pub_name,sub_name):
        self. state =state # x y theta
        self. pos_map =pos_map # x,y real position of each cell
        self. sensor_spec =sensor_spec # [0] = sensor radius / [1] = sensor angle

        # this is for target recoginition
        # In real implementation, we decide whether the platform found target
        self. target =target_idx

        # setpoint : scalar index of pos_map

        self. setpoint =None
        # is this agent for exploration? or tracking?
        # mode is either 'explore' or 'track'
        self. mode =mode

        # this is topic name of this publisher
        # because we have multiple platform, we should give repsectively
        self.pub_name=pub_name
        self.sub_name=sub_name
        # we should match each topic with each platform
        self.pub=rospy.Publisher(pub_name,Pose2D,queue_size=10)
        self.sub=rospy.Subscriber(sub_name,Pose2D,self.callback)



    def callback(self,data):
        self.state[0]=data.x
        self.state[1]=data.y
        self.state[2]=data.theta


    def publish(self):
        pose2d=Pose2D()
        pose2d.x=self.pos_map[self.setpoint][0]
        pose2d.y = self.pos_map[self.setpoint][1]
        pose2d.theta = 0 # just xy point


        # print 'published data: {},{}'.format(pose2d.x,pose2d.y)

        self.pub.publish(pose2d)



        # publish obseraved cell (this is to be done by SLAM team)
    # Therefore below function is just to test the feasiblity of algorithm
    def obs_data(self ,mode):
        # if we use mode 1, this function will give sensored cells
        # if we use mode 2, this will give target candidate cells
        if mode== 1:
            rad, ang = (self.sensor_spec[0], self.sensor_spec[1])
        else:
            rad, ang = (2 * self.sensor_spec[0], self.sensor_spec[1])

        data_idx = []  # observed cell and no target spotted
        target_idx = []  # cell index where the target found

        # cell iteration
        for i in range(len(self.pos_map)):
            cur_pos = self.pos_map[i]
            obs_dist = np.linalg.norm(self.state[0:2] - np.array(cur_pos))
            # theta of agent (0 to 2pi)
            theta = self.state[2]

            # don't mind belows
            # ---------------------------------------------
            if theta < 0:
                theta += 2 * np.pi

            sens_ang = np.arctan2(-self.state[1] + cur_pos[1],
                                  -self.state[0] + cur_pos[0])

            if sens_ang < 0:
                sens_ang += 2 * np.pi
            # ---------------------------------------------

            # measured angle of cell position w.r.t agent coordinate origin
            obs_angle = abs(theta - sens_ang)

            if obs_angle > np.pi:
                obs_angle = 2 * np.pi - obs_angle

            # if this cell is included in sensor range
            if (obs_dist < rad) & (obs_angle < ang / 2):
                # we can observe the cell , will give info to master class
                data_idx.append(i)
                if i == self.target:  # if target found
                    target_idx.append(i)
                    data_idx.remove(i)
        return (data_idx, target_idx)

    def isreached(self):
        # is the target cell within sensor range
        target_idx = self.setpoint
        sensored_idx = self.obs_data(1)[0]
        return target_idx in sensored_idx

    # this functions are not needed in ROS implementation

    def following(self, Kx, Ktheta, dt, verbose):

        theta = self.state[2]
        x = self.pos_map[self.setpoint][0]
        y = self.pos_map[self.setpoint][1]

        theta_target = np.arctan2(y - self.state[1], x - self.state[0])
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

        cmd = np.array((Kx * (x - self.state[0]),
                        Kx * (y - self.state[1]),
                        Ktheta * (cmd_theta)))
        self.state += cmd * dt

    def draw_agent(self):
        for theta in linspace(self.state[2] - self.sensor_spec[1] / 2.0, self.state[2] + self.sensor_spec[1] / 2.0, 9):
            x = [self.state[0], self.state[0] + self.sensor_spec[0] * np.cos(theta)]
            y = [self.state[1], self.state[1] + self.sensor_spec[0] * np.sin(theta)]
            plt.plot(x, y, c='r', alpha=0.2)
        plt.scatter(self.state[0], self.state[1], c='b', marker='o', s=100)


class Prob_map:
    def __init__(self, pos_map, dx, dy,Nx,Ny):
        self.position_map = pos_map
        self.probability_map = [1.0 / len(pos_map)] * len(pos_map)
        self.observation_map = [False] * len(pos_map)
        self.dx, self.dy = (dx, dy)
        self.Nx, self.Ny = (Nx, Ny)

    def updata_map(self, data):
        for i in data[0]:
            # observed and target is not found
            self.probability_map[i] = 0;
        for i in data[1]:
            # observed and target is found
            self.probability_map[i] = 0.9;
        self.probability_map = self.probability_map / sum(self.probability_map)

    def plot_map(self, ax):
        X = [];
        Y = [];
        Z = [];

        for i in range(len(self.position_map)):
            X.append(self.position_map[i][0])
            Y.append(self.position_map[i][1])
            Z.append(self.probability_map[i])

        plt.pcolor(np.array(X).reshape(self.Nx,self.Ny), np.array(Y).reshape(self.Nx,self.Ny),
                   np.array(Z).reshape(self.Nx,self.Ny), alpha=0.3, cmap='jet')

        plt.show()


class Master:
    def __init__(self, pos_map, prob_map, agent_list):
        self.pos_map = pos_map
        self.prob_map = prob_map
        self.agent_list = agent_list

        for agent in agent_list:
            agent.setpoint = np.random.choice(agent.obs_data(mode=2)[0])

    # most important function of the master class
    def waypoint(self, this_agent):

        other_agent_list = list(set(self.agent_list) - set([this_agent]))

        ####################################
        # phase 1 : find waypoint candidates
        ####################################

        # if this agent is explorer, it is nearsighted
        flag = None
        if this_agent.mode == 'explore':

            dists = []
            # sensed cell index list + not sensed yet but lies slightly farther from
            # sensor range
            sensored_idx = this_agent.obs_data(mode=2)[0]
            # corresponding probability map list
            prob_map_sensored = self.prob_map.probability_map[sensored_idx]

            arr = np.array(prob_map_sensored)


            if max(prob_map_sensored) == 0.0:
                # if there is no nonzero probablity cell
                # agent can be lost. so it should become far-sighted

                # global max cell
                cndids = np.where(self.prob_map.probability_map == max(self.prob_map.probability_map))[0]
                flag = 'global'



            else:

                max_idx = np.where(arr == max(arr))[0]
                flag = 'local'
                sensored_idx = np.array(sensored_idx)
                cndids = sensored_idx[max_idx]


        else:  # this_agent.mode == 'track':
            cndids = np.where(self.prob_map.probability_map == max(self.prob_map.probability_map))[0]
            flag = 'global'

        # something wrong


        #################################################
        # phase 2 : find waypoint considering other agent
        #################################################

        other_setpoint = []
        for agent in other_agent_list:
            other_setpoint.append(agent.setpoint)  # this is current setpoint of others

        if flag == 'global':
            if np.random.rand() < 0.9:
                # global max cell but especially farthest from other setpoint



                max_cost = 0
                max_idx = None

                for idx in cndids:
                    cost = 10/np.linalg.norm(self.pos_map[idx] - this_agent.state[0:2])



                    for other_setpoint_idx in other_setpoint:
                        cost += np.linalg.norm(self.pos_map[idx] - self.pos_map[other_setpoint_idx])
                    if cost > max_cost:
                        max_cost = cost
                        max_idx = idx

                target_idx = (max_idx)
                this_agent.setpoint = target_idx
            else:

                this_agent.setpoint = np.random.choice(cndids)

            # return (self.pos_map[target_idx],target_idx)
            return
        else:  # flag= local

            # with a half probability, local max that has maximum cost

            if np.random.rand() < 0.5:
                # cost is the total of distance between other setpoints
                max_cost = 0
                max_idx = None

                for idx in cndids:
                    cost = 0
                    for other_setpoint_idx in other_setpoint:
                        cost += np.linalg.norm(self.pos_map[idx] - self.pos_map[other_setpoint_idx])
                    if cost > max_cost:
                        max_cost = cost
                        max_idx = idx

                this_agent.setpoint = max_idx
                # return (self.pos_map[max_idx],max_idx)
                return

                # with a half probability, random choice among local maxes
            else:
                tgt_idx = np.random.choice(cndids)
                this_agent.setpoint = tgt_idx
                # return (self.pos_map[tgt_idx],tgt_idx)
                return

                # every step, prob_map should be updated

    def prob_map_update(self):
        for agent in self.agent_list:
            self.prob_map.updata_map(agent.obs_data(1))



