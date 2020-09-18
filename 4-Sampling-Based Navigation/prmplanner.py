# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import numpy as np
from scene import Scene
from obstacles import BoxObstacle
import matplotlib.pyplot as plt
from scipy.spatial import distance as dis

disk_robot = True #(change this to False for the advanced extension)
N_samples = 1000
obstacles = None # the obstacles
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius, N_samples

    obstacles = scene_obstacles # setting the global obstacle variable
    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)
    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    robot_radius = max(robot_width, robot_height)/2

    candidate_stack = np.zeros((N_samples,2))

    candidate_stack = perturb(N_samples,x_limit,y_limit) #sampled number

    graph = Roadmap() # the roadmap
    candidate_rem = []
    for pos,val in enumerate(candidate_stack):
        for obj in obstacles:
            xs = []
            ys = []
            for p in obj.points:
                xs.append(p[0])
                ys.append(p[1])
            x_min=min(xs) - robot_radius
            x_max=max(xs) + robot_radius
            y_min=min(ys) - robot_radius
            y_max=max(ys) + robot_radius
            if (val[0]>x_min and val[0]<x_max and val[1]>y_min and val[1]<y_max):
                candidate_rem.append(candidate_stack[pos])
    candidate_rem = np.asarray(candidate_rem)

    cand_stack_x = ExtractX(candidate_stack)
    cand_stack_y = ExtractY(candidate_stack)
    cand_rem_x = ExtractX(candidate_rem)
    cand_rem_y = ExtractY(candidate_rem)

    differencesX = []

    for list in cand_stack_x:
        if list not in cand_rem_x:
            differencesX.append(list)

    differencesY = []

    for list in cand_stack_y:
        if list not in cand_rem_y:
            differencesY.append(list)

    cand_final = []
    h = []

    for i in range(len(differencesX)):
        cand_final.append([differencesX[i],differencesY[i]])

    for k in range(len(cand_final)):
        graph.addVertex((cand_final[k][0],cand_final[k][1]))

    for vertex in graph.vertices:
        for child in graph.vertices:
            c = 0
            if vertex!=child:
                #distance=math.sqrt((vertice.q[0]-poss_neighbor.q[0])**2 + (vertice.q[1]-poss_neighbor.q[1])**2)
                dist_2 = distance(vertex.q,child.q)
                if dist_2 <= 10:
                    h = interpolate(vertex.q,child.q,5)
                    for ho in h:
                        for o in range(len(obstacles)):
                            if ((obstacles[o].x_min-robot_radius<=ho[0]<=obstacles[o].x_max+robot_radius) and (obstacles[o].y_min-robot_radius<=ho[1]<=obstacles[o].y_max+robot_radius)):
                                c = 1
                    if c == 0:
                        #graph.addEdge(vertice,poss_neighbor,distance)
                        if child.connectedComponentNr == -1 and child.getEdge(vertex.id)==None:
                            #vertice.addEdge(poss_neighbor.id,distance)
                            graph.addEdge(vertex,child,dist_2)
                            child.connectedComponentNr = vertex.id
                        else:
                            path_parent = find(vertex,graph.vertices)
                            path_child = find(child,graph.vertices)
                            path_length = len(set(path_child) & set(path_parent))
                            if path_length == 0 and child.getEdge(vertex.id)==None:
                                graph.addEdge(vertex,child,dist_2)
                                child.connectedComponentNr = vertex.id

    #print(Scene.default_start.get(loadProblem))
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    global obs
    path  = []
    heuristic = []

    start = graph.addVertex((q_start[0], q_start[1]))   # Add start vertex
    goal = graph.addVertex((q_goal[0], q_goal[1]))     # Add goal vertex

    for nodes in graph.getVertices():
        heuristic.append(distance(goal.q,nodes.q))

    m = []
    l = []

    obs.getObstacles()  # Call the getObstacles method from scene.py
    for vert in graph.vertices:
        c = 0
        d = 0
        if (distance(q_start,vert.q)) <= 6: # Distance between start and all nodes
            m = interpolate(start.q,vert.q,5)
            for go in m:
                for o in range(len(obstacles)):
                    if ((obstacles[o].x_min-robot_radius<=go[0]<=obstacles[o].x_max+robot_radius) and (obstacles[o].y_min-robot_radius<=go[1]<=obstacles[o].y_max+robot_radius)):
                        c = 1
                if c == 0:
                    graph.addEdge(vert,start, distance(q_start,vert.q)) # Add edge between start vertex and its nearby vertices
        if (distance(q_goal,vert.q)) <= 6: # Distance between goal and all nodes
            l = interpolate(goal.q,vert.q,5)
            for lo in l:
                for o in range(len(obstacles)):
                    if ((obstacles[o].x_min-robot_radius<=lo[0]<=obstacles[o].x_max+robot_radius) and (obstacles[o].y_min-robot_radius<=lo[1]<=obstacles[o].y_max+robot_radius)):
                        d = 1
                if d == 0:
                    graph.addEdge(vert,goal, distance(q_goal,vert.q))  # Add edge between goal vertex and its nearby vertices

    parent =  np.empty(graph.getNrVertices(), dtype = (tuple,2))
    start_2d = (q_start[0], q_start[1])
    goal_2d = (q_goal[0], q_goal[1])

    start_heuristic = [i for i in range(len(heuristic)) if heuristic[i] == math.sqrt((q_start[0] - q_goal[0])**2 + (q_start[1] - q_goal[1])**2)]
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    graph.edges = graph.getVertices()
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    x = q_start[0]
    y = q_start[1]
    theta = q_start[2]
    h_start = heuristic[start_heuristic[0]]
    g = 0
    f = g+h_start
    open_set.put(q_start, Value(f=f,g=g))
    final = goal_2d

    # your code: implement A*
    # Implementing A*
    while len(open_set)!=0:
        current_node,current_node_add=open_set.pop()
        current_node_2d = (current_node[0],current_node[1])
        closed_set.add(current_node)
        if current_node[0]==q_goal[0] and current_node[1]==q_goal[1]:
            print("Goal reached")
            # Tracing path back from goal to start
            while final[0] != start_2d[0] and final[1] != start_2d[1] :
                for vertice in graph.vertices:
                    if vertice.q[0] == final[0] and vertice.q[1] == final[1]:
                        back = parent[vertice.id]
                        path.append(back)
                        final = back
                        break
            break
        for vertice in graph.vertices:
            if vertice.q[0] == current_node_2d[0] and vertice.q[1] == current_node_2d[1]:
                for edge in vertice.edges:
                    child_node = (graph.vertices[edge.id].q[0], graph.vertices[edge.id].q[1])
                    if child_node in closed_set:
                        continue
                    else:
                        g_child = current_node_add.g + edge.dist
                        if child_node not in open_set or open_set.get(child_node).g > g_child:
                            f_child = g_child + heuristic[edge.id]
                            open_set.put(child_node, Value(f_child,g_child))
                            parent[edge.id] = (current_node[0],current_node[1])

    path.reverse()  # This is being used to reverse the path so that the dots go from blue to green as we progress towards the goal node
    return path


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """

    return None, None


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """

    return None

def distance (q1, q2):
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """
    dist = math.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)
    return dist

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
    """
    """if q.all() in obstacles:
        yes = Roadmap.removeVertex(i)
    else:
        yes = None"""

    return False


def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """
    x_spacing = (q2[0] - q1[0]) / (stepsize+1)
    y_spacing = (q2[1] - q1[1]) / (stepsize+1)

    return [[q1[0] + i * x_spacing, q1[1] +  i * y_spacing]
            for i in range(1, stepsize+1)]

def ExtractX(lst):
    return [item[0] for item in lst]

def ExtractY(lst):
    return [item[1] for item in lst]

def perturb(n,x_limit,y_limit):

    limx = np.int(np.sqrt(n)) + 1
    limy = np.int(n / limx) + 1

    x = np.linspace(x_limit[0], x_limit[1], limx, dtype=np.float)
    y = np.linspace(y_limit[0], y_limit[1], limy, dtype=np.float)
    samp = np.stack(np.meshgrid(x, y), -1).reshape(-1,2)

    dist_init = np.min((x[1]-x[0], y[1]-y[0]))
    dist_min = dist_init * 0.45

    max_move = (dist_init - dist_min)/2
    pert = np.random.uniform(low=-max_move, high=max_move,size=(len(samp), 2))
    samp = samp + pert

    return samp

def find(node, vertex): # implement this to compute the connected components of the undirected graph
    n = node
    paths = []
    roads = []
    count = 0
    count1 = 0
    while(n.connectedComponentNr!=-1):
        parent=n.connectedComponentNr
        paths.append(parent)
        for v in vertex:
            if parent == v.id:
                count=v
        n=count
    parent=n.id
    paths.append(parent)
    for p in paths:
        for vv in vertex:
            if vv.id == p:
                count1=vv
        paths=count1.getEdges()
        for pp in paths:
            x=pp.src_id
            roads.append(x)
            y=pp.dest_id
            roads.append(y)
    return roads


if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    obs = Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
