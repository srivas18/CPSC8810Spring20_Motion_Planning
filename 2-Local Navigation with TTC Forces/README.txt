CPSC 8810 - Motion Planning
ASSIGNMENT 2: Local Navigation with TTC Forces
Submitted by: Aakanksha Smriti, Siddhant Srivastava
Submission date: 02-09-2020

Variable Description:
1. size = The sample size over which random velocities are generated
2. samp_rad = Saving the self.maxspeed as sampling radius
3. theta = Creating the random sampling angle for velocities
4. r = Creating the random sampling velocities inside the sampling disk area
5. V = Array to store the x and y components of the sample of candidates
6. goal_p = Euclidean distance between goal and agent positions
7. nearest_neighbor_id = Array to store nearest neighbor id
8. nearest_neighbor_pos = Array to store nearest neighbor positons
9. nearest_neighbor_rad = Array to store nearest neighbor radius
10. nearest_neighbor_vel = Array to store nearest neighbor velocity
11. dis_test = Euclidean distance between candidate and agent positions
12. nearest_neighbor_num = Number of nearest neighbors
13. V_select = To store the minimum candidate velocity with minimum cost function
14. Fg = Goal force
15. e = Uncertainty in sensed velocities
16. dh = Distance horizon
17. k = Scaling factor for Power Law model
18. to = Exponential cut-off point in Power Law model
19. m = Constant in Power Law model
20. Favoid = Avoidance force
21. n = Unit vector of the forces
