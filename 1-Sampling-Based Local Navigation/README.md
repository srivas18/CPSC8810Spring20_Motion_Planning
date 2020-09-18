# CPSC 8810 - Motion Planning
## ASSIGNMENT 1: Sampling-Based Local Navigation
###### Submitted by: Aakanksha Smriti, Siddhant Srivastava
###### Submission date: 01-26-2020

## Variable Description:
1. size = The sample size over which random velocities are generated
2. samp_rad = Saving the self.maxspeed as sampling radius
3. theta = Creating the random sampling angle for velocities
4. r = Creating the random sampling velocities inside the sampling disk area
5. V_cand = Array to store the x and y components of the sample velocities of candidates
6. goal_p = Euclidean distance between goal and agent positions
7. nearest_neighbor_id = Array to store nearest neighbor id
8. nearest_neighbor_pos = Array to store nearest neighbor positons
9. nearest_neighbor_rad = Array to store nearest neighbor radius
10. nearest_neighbor_vel = Array to store nearest neighbor velocity
11. dis_test = Euclidean distance between candidate and agent positions
12. nearest_neighbor_num = Number of neaest neighbors
13. Cost_func = Array to store the cost functions
14. V_select = To store the minimum candidate velocity with minimum cost function

## Instruction:
Run `python agent.py` to execute the program.
Change `.csv` file at line 21 of `simulator.py` from 3 agents to 8 agents
