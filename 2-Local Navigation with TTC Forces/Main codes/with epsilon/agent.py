# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt
import math

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
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
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent


    def computeForces(self, neighbors=[]):
        """
            Your code to compute the forces acting on the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """
#=======================================================================================================================
# Calculation of Goal Force (Fg), Sensing Distance, All neighbors, Avoidance Force (Fa) and TTC
#=======================================================================================================================
        e = 0.2
        dh = 10
        Fg = (self.gvel-self.vel)/self.ksi

        # Calculating sensing distance and all neighbors
        for j in range(len(neighbors)):
            if self.id != neighbors[j].id and not neighbors[j].atGoal:
                dis_test = sqrt((neighbors[j].pos[0]-self.pos[0])**2 + (neighbors[j].pos[1]-self.pos[1])**2) - (self.radius + neighbors[j].radius)
                if(dis_test < dh):
                    # Calculating TTC
                    r = neighbors[j].radius + self.radius
                    w = self.pos - neighbors[j].pos #relative position
                    c = w.dot(w) - r*r
                    rel_vel = self.vel - neighbors[j].vel
                    # Accounting for uncertainty in the sensed velocities 'e'
                    a = np.dot(rel_vel,rel_vel) - (e**2)
                    b = np.dot(rel_vel,w) - (e*r)
                    discr = b*b - a*c
                    if b > 0:
                        ttc = float('inf')
                    if discr <= 0:
                        ttc = float('inf')
                    if discr > 0:
                        tau = c / (-b + sqrt(discr))
                        if tau < 0:
                            ttc = float('inf')
                        else:
                            ttc = tau
                    if (ttc != float('inf')): # Calculating unit vector of avoidance force 'n'
                        n = w+(rel_vel*ttc)
                    else:
                        n = np.zeros(2)
                    if (n[0] != 0 and n[1] != 0):
                        n = n/sqrt(n.dot(n))

                    Favoid = max(5 - ttc,0)/ttc # Calculating magnitude of avoidance force

                    n = n * Favoid # Calculating avoidance force
                    Fg = Fg + n

                    # Capping the total force applied to the agent
                    if not self.atGoal:
                        self.F = Fg
                        self.F = np.clip(self.F, -self.maxF, self.maxF)

            else:
                self.F = Fg
                self.F = np.clip(self.F, -self.maxF, self.maxF)

#=======================================================================================================================
# Capping the velocity of each agent to its maximum speed
#=======================================================================================================================

    def update(self, dt):
        """
            Code to update the velocity and position of the agents.
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.vel = np.clip(self.vel, -self.maxspeed, self.maxspeed)
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
