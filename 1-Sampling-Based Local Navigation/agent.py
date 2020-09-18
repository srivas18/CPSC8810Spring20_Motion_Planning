# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


import numpy as np
from math import*
import matplotlib.pyplot as plt
from scipy.spatial import distance as dis

class Agent(object):
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent


    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
#=======================================================================================================================
# Uniformly sampling candidate velocities for 200 sample in the AV space
#=======================================================================================================================
        size = 200 #Sample size N = 200
        samp_rad = self.maxspeed #sampling radius R
        theta = np.random.uniform(0,1,size)*pi*2 #uniformly distributing theta between 0 to 2pi
        r = np.sqrt(np.random.uniform(0,1,size)) * samp_rad #r = sqrt.u * R

        V_cand = np.zeros((size,2)) #Considering x and y velocity components of 100 candidates
        V_cand[:,0] = r*np.cos(theta) #x-component of candidate velocity
        V_cand[:,1] = r*np.sin(theta) #x-component of candidate velocity

#=======================================================================================================================
# Finding the nearest neighbors
#=======================================================================================================================
        goal_p = np.linalg.norm(self.goal-self.pos) # Euclidean distance between goal and agent positions

        if goal_p > 1: #condition for agent being at least 1m away from its goal position (Part 5)
            nearest_neighbor_id = []
            nearest_neighbor_pos = []
            nearest_neighbor_rad = []
            nearest_neighbor_vel = []
            for neighbor in neighbors:
                if self.id != neighbor.id:
                    dis_test = dis.euclidean(self.pos,neighbor.pos)
                    if(dis_test < self.dhor):
                        nearest_neighbor_id.append(neighbor.id)
                        nearest_neighbor_pos.append(neighbor.pos)
                        nearest_neighbor_rad.append(neighbor.radius)
                        nearest_neighbor_vel.append(neighbor.vel)
            nearest_neighbor_num = len(nearest_neighbor_id)


#=======================================================================================================================
# Evaluating the fitness of each candidate in the sample space by tuning the cost function parameters
#=======================================================================================================================
            Cost_func = np.zeros(size)
            if nearest_neighbor_num!=0:
                for i in range(size):
                    ttc = np.zeros(len(nearest_neighbor_id)) #TTC of the nearest neighbors
                    for j in range(len(nearest_neighbor_id)):
                        r = nearest_neighbor_rad[j] + self.radius
                        w = self.pos - nearest_neighbor_pos[j] #relative position
                        c = np.dot(w,w) - r*r
                        if c < 0:
                            ttc[j] = 0
                        rel_vel = V_cand[i] - nearest_neighbor_vel[j]
                        a = np.dot(rel_vel,rel_vel)
                        b = np.dot(rel_vel,w)
                        discr = b*b - a*c
                        if b > 0:
                            ttc[j] = float('inf')
                        if discr <= 0:
                            ttc[j] = float('inf')
                        else:
                            tau = c / (-b + sqrt(discr))
                            if tau < 0:
                                ttc[j] = float('inf')
                            else:
                                ttc[j] = tau

#Computing tc from time to collision (ttc)
                    tc = np.amin(ttc)
                    alpha = 10
                    beta = 10
                    gamma = 20
                    Cost_func[i] = alpha*np.linalg.norm(V_cand[i] - self.gvel) + beta*np.linalg.norm(V_cand[i] - self.vel) + gamma/tc
                Cost_func_min = np.amin(Cost_func)
#Selected velocity as the index of minimum cost function
                V_select = V_cand[np.argmin(Cost_func)]

#The new velocity will be the selected velocity
                if not self.atGoal:
                    self.vnew[:] = V_select[:]

            elif nearest_neighbor_num == 0 and not self.atGoal:
                self.vnew[:]=self.gvel[:]

#=======================================================================================================================
# Main execution
#=======================================================================================================================
    def update(self, dt):
        """
            Code to update the velocity and position of the agent
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
