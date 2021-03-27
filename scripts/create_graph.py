import numpy as np
import math
import matplotlib.pyplot as plt
from PIL import Image
import random
import time 
import sys
import pickle
import resource
import os
import yaml

####################################################
#           table of contents
#
#           Global Parameters
#           vertex class
#           help functions
#           kin update functions
#           RRT functions
#           RRT graph
#           a_star
#           visualization functions
#           init functions
#           main
#           launch main
####################################################  

####################################################
#           Global Parameters
####################################################  

GAME_x = (0, 40) #map boarders
GAME_y = (0, 40) #map boarders
lim = 300 #count of vertex
count = 1 #first_vertex=0
obs = np.empty((0,2)) #array of obs
graph = np.empty(lim, dtype=object) #array of vertices
xy_graph = np.zeros((0, 2))
max_cost_2_goal = 2.5 #maximum cost to goal
rad_obs = 0.25 #my min dist to an obs that its valid
rad_search = 1 #search radius vor near vertex
step = 1 #max step size
start = None #start vertex, id = len(graph)
goal = None #goal vertex, id = len(graph)+1
resolution = 0.05 #map param to load pgm
shift_x = 200 #map param to load pgm
shift_y = 200 #map param to load pgm
k_phi = 1.2 #positive constant that represent the weight of phi with respect to r.
k_delta = 10 #positive constant that represent the weight of delta with respect to phi.



####################################################
#           vertex class
####################################################

class vertex:

    def __init__(self, id, xyp):
        global graph
        self.id = id
        self.xyp = xyp
        self.neighbors = [] #from self to neighbor (v, cost)
        
        #params for a_star
        self.parent = None
        self.g = None #previous costs from the start node
        self.h = None #estimated costs to the goal node
        self.f = None #node with the lowest f-value is examined next (f = g + h)
        
        if id >= lim:
            graph = np.append(graph, self)
        else:
            graph[id] = self #add vertex to graph
       
    def add_2_neighbors(self, v, cost):
        self.neighbors.append((v, cost))
        
    def print_vertex(self):
        print("ID: ", self.id)
        print("xyp: ", self.xyp)



####################################################
#           help functions
####################################################  

def wrap_to_pi(angle):
	wrappedAngle = angle %  math.pi
	times = angle//math.pi
	if (times % 2) == 1: # ungerade
		wrappedAngle = wrappedAngle - math.pi 
	return wrappedAngle

# compute robot pose in x y phi
def convert_rpd_2_xyp(goal_pose, rpd_pose): 
	los_angle = goal_pose[2] - rpd_pose[1]
	x = goal_pose[0] - rpd_pose[0]*math.cos(los_angle)
	y = goal_pose[1] - rpd_pose[0]*math.sin(los_angle)
	phi = wrap_to_pi(los_angle + rpd_pose[2])
	return [x, y, phi]

# compute robot pose in radius phi delta
def convert_xyp_2_rpd(robot_pose, goal_pose): 
	delta_x = goal_pose[0] - robot_pose[0]
	delta_y = goal_pose[1] - robot_pose[1]
	r = math.sqrt(delta_x**2 + delta_y**2) # if r small, use goal_pose[2]
	if r < 0.1:
		los_angle = goal_pose[2]
	else:
		los_angle = math.atan2(delta_y, delta_x) # angle of line-of-sight
	phi = wrap_to_pi(goal_pose[2] - los_angle)
	delta = wrap_to_pi(robot_pose[2] - los_angle)
	return [r, phi, delta]


#euclidean distance
def eucl_dist(coord1, coord2):
    delta_x = coord2[0] - coord1[0]
    delta_y = coord2[1] - coord1[1]
    return np.sqrt(delta_x**2 + delta_y**2)

#coordinates are on a obs
def valid_pose(xyp):
    if np.linalg.norm(obs-xyp[:2], axis=1).min() < rad_obs:
        return False
    if xyp[0] < GAME_x[0] or xyp[1] < GAME_y[0] or xyp[0] > GAME_x[1] or xyp[1] > GAME_y[1]: #out of bounds?
        return False
    return True

#random coordinates on the map
def random_pose():
    param = 0.01 #number of decimal places of random coordinates
    x = random.randrange(GAME_x[0]/param, GAME_x[1]/param) * param
    y = random.randrange(GAME_y[0]/param, GAME_y[1]/param) * param
    phi = random.randrange(-3.14/param, 3.14/param) * param
    return (x, y, phi)

#returns True if v is near the goal
def reached_goal(v):
    if v == None:
        return False
    if dist(v.xyp, goal.xyp) < max_cost_2_goal and valid_link(v.xyp, goal.xyp):
        return True
    else:
        return False

#returns true if xyp1 is near xyp2
def near_to(xyp1, xyp2, rad):
    if abs(xyp1[0] - xyp2[0]) < rad and abs(xyp1[1] - xyp2[1]) < rad:
        return True
    else:
        return False

#adds x y coordinates to the xy_graph
def add_2_xy_graph(v_new):
    global xy_graph
    xy_graph = np.append(xy_graph, [v_new.xyp[:2]], axis=0)



####################################################
#           kin update functions
####################################################  

def kin_update_xyp(pose, v, omega, delta_t):
	phi = pose[2] + omega * delta_t
	x = pose[0] + math.cos(phi) * v * delta_t
	y = pose[1] + math.sin(phi) * v * delta_t
	return [x, y, phi]

def smooth_control(rpd): # ICRA2011: A Smooth Control Law for Graceful Motion of Diff...
# TODO: add parameters to input arguments
	v_max = 0.22 # 1.2 # wheelchair;  turtlebot 0.22
	#beta = 0.4
	#lambd = 2
	rpd_r = rpd[0]
	rpd_phi = rpd[1]
	rpd_delta = rpd[2]
	bracket = k_delta * (rpd_delta - math.atan(-1 * k_phi * rpd_phi)) + (1+k_phi/(1+(k_phi*rpd_phi)**2)*math.sin(rpd_delta))
	omega = -1 * v_max / rpd_r * bracket
	if abs(omega) > 0.9: # 0.9 is max(omega_wheelchair_user)
		omega = np.sign(omega)*0.9
	#curvature_kappa = -1 / rpd_r * bracket
	v = v_max #/ (1 + beta* abs(curvature_kappa)**lambd)
	#v = v_from_omega(omega)
	return v, omega



####################################################
#           RRT functions
####################################################  

#cost function
def dist(xyp_from, xyp_to):
    rpd = convert_xyp_2_rpd(xyp_from, xyp_to)
    l_minus = np.sqrt((rpd[0]**2) + (k_phi**2) * (rpd[1]**2))
    delta_e = abs( rpd[2] - math.atan(-1 * k_phi * rpd[1]) )
    return l_minus + k_delta * delta_e

# coordinates of the nearest vertex
def nearest_vertex(xyp):
    return graph[np.linalg.norm(xy_graph-xyp[:2], axis=1).argmin()]

#returns true if the link between xyp_from and xyp_to is collsion free
def valid_link(xyp_from, xyp_to):
    delta_t = 1.5
    if not valid_pose(xyp_to):
        return False
    if eucl_dist(xyp_from, xyp_to) == 0:
        return False
    xyp_state = xyp_from
    while not near_to(xyp_state, xyp_to, rad_obs):
        rpd_state = convert_xyp_2_rpd(xyp_state, xyp_to)
        v, omega = smooth_control(rpd_state)
        xyp_state = kin_update_xyp(xyp_state, v, omega, delta_t)
        if not valid_pose(xyp_state):
            return False
    return True

#returns vertex coordinates step away from nearest.xyp in direction to xyp_new
def steer(nearest, xyp_new):
    if eucl_dist(nearest.xyp, xyp_new) == 0:
        return (-1, -1, 0)
    delta_x = (xyp_new[0] - nearest.xyp[0]) * step / eucl_dist(nearest.xyp, xyp_new)
    delta_y = (xyp_new[1] - nearest.xyp[1]) * step / eucl_dist(nearest.xyp, xyp_new)
    xyp_res = (nearest.xyp[0] + delta_x, nearest.xyp[1] + delta_y, xyp_new[2])
    if eucl_dist(nearest.xyp, xyp_new) <= eucl_dist(nearest.xyp, xyp_res):
        xyp_res = xyp_new
    if not valid_link(nearest.xyp, xyp_res):
        return (-1, -1, 0)
    else:
        return xyp_res

#finds all vertex that are closer than rad_search
def find_neighbors(v_new):
    return graph[np.where(np.linalg.norm(xy_graph-v_new.xyp[:2], axis=1) < rad_search)]

#finds all vertex that are closer than rad_search
def find_neighbors_without_xy_graph(v_new):
    neighbors = []
    for v in graph:
        if near_to(v_new.xyp, v.xyp, rad_search) and v_new.id != v.id:
            neighbors.append(v)
    return neighbors

#links v_new with all neighbors (inclusiv cost)
def link_neighbors(v_new, neighbors):
    for neighbor in neighbors:
        if valid_link(v_new.xyp, neighbor.xyp):
            v_new.add_2_neighbors(neighbor, dist(v_new.xyp, neighbor.xyp))
        if valid_link(neighbor.xyp, v_new.xyp):
            neighbor.add_2_neighbors(v_new, dist(neighbor.xyp, v_new.xyp))



####################################################
#           RRT graph
####################################################  

def RRT_star():
    global count
    while count < lim:
        xyp_new = random_pose() #new coordinates
        nearest = nearest_vertex(xyp_new) #nearest vertex
        xyp_new = steer(nearest, xyp_new) #vertex step away in direction of xyp_new
        if xyp_new == (-1, -1, 0): #not a valid vertex
            continue
        
        v_new = vertex(count, xyp_new) #append in graph
        
        neighbors = find_neighbors(v_new) #finds near vertices in rad_search
        
        link_neighbors(v_new, neighbors) #link v_new with all neighbors (inclusiv cost)
        
        add_2_xy_graph(v_new)
        
        count = count+1
        update_progress(float(count)/float(lim)) #update progress bar



####################################################
#           a_star
####################################################  

#init lists
open_list = [] #list with unvisited vertices
closed_list = [] #list with visited vertices

def a_star():
    #init
    start.g = 0
    start.f = dist(start.xyp, goal.xyp)
    open_list.append(start)
    
    for v in graph:
        v.h = dist(v.xyp, goal.xyp)
    
    #loop until found goal
    while len(open_list) > 0:
    
        # Get the current node with lowest f-value
        current_vertex = None
        min_f = np.inf
        for v in open_list:
            if v.f < min_f:
                min_f = v.f
                current_vertex = v
        open_list.remove(current_vertex)
        
        #found goal
        if current_vertex.id == goal.id:
            return True
        
        #the current_vertex should not be examined further to avoid loops
        closed_list.append(current_vertex)
        
        #Add successor vertices of the current vertex to the Open List
        expandVertex(current_vertex)
    
    return False

#checks all successor vertices and adds them to the Open List if either
# - the successor vertex is found for the first time, or
# - a better way to this vertex is found
def expandVertex (current_vertex):
    for neighbor in current_vertex.neighbors: #neighbor = (neighbor, cost_2_neighbor)
        #if the successor vertex is already on the closed list - do nothing
        if neighbor[0] in closed_list:
            continue
        
        #calc g-value for the new path: g-value of the predecessor plus the cost of the currently used vertex
        new_g_of_neighbor = current_vertex.g + neighbor[1]
        
        #successor vertex is already in the Open list and the old path is better - do nothing
        if neighbor[0] in open_list and new_g_of_neighbor >= neighbor[0].g:
            continue
        
        #set parent and g-value
        neighbor[0].parent = current_vertex
        neighbor[0].g = new_g_of_neighbor
        
        #update f-value of the vertex
        neighbor[0].f = new_g_of_neighbor + neighbor[0].h
        
        #vertex is not in the Open list - append
        if not neighbor[0] in open_list:
            open_list.append(neighbor[0])

#returns the trace from start to goal
def get_trace():
    trace = []
    v_tmp = goal
    while v_tmp.id != start.id:
        trace.append(v_tmp)
        v_tmp = v_tmp.parent
    
    #turn trace around (trace[0] = Start)
    res = []
    res.append(start)
    for i in range(len(trace)-1, -1, -1):
        res.append(trace[i])
    return res


####################################################
#           visualization functions
####################################################  

#plots an arrow out of xyp
def plot_arrow(xyp, color):
    arrow_length = 0.1
    dx = arrow_length * np.cos(xyp[2])
    dy = arrow_length * np.sin(xyp[2])
    plt.arrow( xyp[0], xyp[1], dx, dy, width = 0.01, color = color )

#plots the dirven path between the points on the trace
def plot_driven_trace(trace, color):
    for i in range(len(trace)-1):
        delta_t = 0.1
        xyp_state = trace[i].xyp
        while not near_to(xyp_state, trace[i+1].xyp, rad_obs):
            rpd_state = convert_xyp_2_rpd(xyp_state, trace[i+1].xyp)
            v, omega = smooth_control(rpd_state)
            xyp_state = kin_update_xyp(xyp_state, v, omega, delta_t)
            plot_arrow(xyp_state, color)

#map/graph plot
def plot_it(trace=None):
    plt.axis('equal')
    x1 = []
    y1 = []
    for o in obs:
        x1.append(o[0])
        y1.append(o[1])
    plt.plot(x1, y1, 'ks')
    
    for v in graph:
        if v != None:
            plot_arrow(v.xyp, 'blue')
        
    if start != None and goal != None:
        plot_arrow(start.xyp, 'red')
        plot_arrow(goal.xyp, 'red')
    
    if trace != None:
        plot_driven_trace(trace, 'yellow')
        
    plt.show()

#loading bar
def update_progress(progress): 
    barLength = 20 # Modify this to change the length of the progress bar 
    status = "" 
    if isinstance(progress, int): 
     progress = float(progress) 
    if not isinstance(progress, float): 
     progress = 0 
     status = "error: progress var must be float\r\n" 
    if progress < 0: 
     progress = 0 
     status = "Halt...\r\n" 
    if progress >= 1: 
     progress = 1 
     status = "Done...\r\n" 
    block = int(round(barLength*progress)) 
    text = "\rPercent: [{0}] {1}% {2}".format("#"*block + "-"*(barLength-block), progress*100, status) 
    sys.stdout.write(text) 
    sys.stdout.flush() 

def print_graph():
        for v in graph[:count]:
            v.print_vertex()
            print("------------------------------")
            
def print_trace(trace):
        for v in trace:
            v.print_vertex()
            print("------------------------------")



####################################################
#           init functions
####################################################    

#loads a img of gazebo slam and selects wall pixel
def init_world(world):
    #load img
    img = Image.open(world)
    n_img = np.asarray(img)
    
    global obs
    for i in range(len(n_img)):
        for j in range(len(n_img[i])):
            if n_img[i][j] == 0:
                near = False
                for o in obs:
                    if near_to(o, ((j-shift_x)*resolution, (abs(i-len(n_img))-shift_y)*resolution), 0.15):
                        near = True
                if not near:
                    obs = np.append( obs, [( (j-shift_x)*resolution, (abs(i-len(n_img))-shift_y)*resolution )], axis=0 )
    
    global GAME_x
    GAME_x = (int(obs.min(axis=0)[0]-1), int(obs.max(axis=0)[0]+1))
    global GAME_y
    GAME_y = (int(obs.min(axis=0)[1]-1), int(obs.max(axis=0)[1]+1))

#defines the first vertex from which the graph expands
def init_first_vertex(xyp):
    v = vertex(0, xyp)
    add_2_xy_graph(v)

#defines the start coordinates of the trace
def init_start(xyp):
    global start
    start = vertex(lim, xyp)
    
    neighbors = find_neighbors_without_xy_graph(start) #finds near vertices in rad_search
    
    link_neighbors(start, neighbors) #link start with all neighbors (inclusiv cost)
    
#defines the goal coordinates of the trace
def init_goal(xyp):
    global goal
    goal = vertex(lim+1, xyp)
    
    neighbors = find_neighbors_without_xy_graph(goal) #finds near vertices in rad_search
    
    link_neighbors(goal, neighbors) #link goal with all neighbors (inclusiv cost)



####################################################
#           main
####################################################  

#calcs the RRT graph
def calc_graph(world, start_xyp, save_file): #(0, 0, 1.57); (0, 0, 0); (0, 0, -1.57)
    init_first_vertex(start_xyp)
    
    global resolution
    global shift_x
    global shift_y
    
    #init_world
    #path to worlds dir
    path = os.path.dirname(os.path.abspath(__file__))
    path = os.path.dirname(path)
    #read yaml file
    add_to_path = 'worlds/' + world + '/' + world + '.yaml'
    yaml_path = os.path.join(path, add_to_path)
    with open(yaml_path, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    origin = [data_loaded["origin"][0], data_loaded["origin"][1]]
    resolution = data_loaded["resolution"]
    shift_x = abs(origin[0]/resolution)
    shift_y = abs(origin[1]/resolution)
    #read pgm file
    add_to_path = 'worlds/' + world + '/' + world + '.pgm'
    path = os.path.join(path, add_to_path)
    init_world(path) #extract obs
    
    if len(obs) == 0:
        print("initialization of the world failed")
        print("choose between world1, world2, world3, world4, world5 and world6")
        return None
    
    plot_it()
    
    #calcs the RRT graph
    time_start = time.time()
    RRT_star()
    time_end = time.time()
    
    #saves the graph
    path = os.path.dirname(os.path.abspath(__file__)) #pwd to file
    path = os.path.dirname(path) #parent dir
    path = os.path.join(path, 'worlds') #final path
    path = os.path.join(path, save_file)
    file = open(path, 'w') # open a binary file in write mode
    pickle.dump(graph, file) # save array to the file
    file.close # close the file
    
    plot_it()
    
    return time_end-time_start

#calcs the trace from start to goal node
def calc_trace(start_xyp, goal_xyp):
    init_start(start_xyp)
    print("start point", start.xyp)
    init_goal(goal_xyp)
    print("goal point", goal.xyp)
    #plot_it()
    if len(start.neighbors) == 0:
        print("start vertex has no neighbors")
        return None, None, 0
    
    time_start = time.time()
    found_trace = a_star()
    time_end = time.time()
    
    if found_trace:
        trace = get_trace()
        return trace, time_end-time_start
    else:
        return None, None, 0

#returns: trace_found?, trace, time_calc_graph, time_calc_trace
def main(world, start_xyp, goal_xyp, n_vertex, r_search, load_saved_graph, calc_new_graph):
    
    print("world: ", world)
    
    save_file = 'graph.obj'
    
    max_rec = 0x100000
    # May segfault without this line. 0x100 is a guess at the size of each stack frame.
    resource.setrlimit(resource.RLIMIT_STACK, [0x100 * max_rec, resource.RLIM_INFINITY])
    sys.setrecursionlimit(max_rec)
    
    time_calc_graph = 0
    time_calc_trace = 0
    
    global lim
    lim = n_vertex
    global graph
    graph = np.empty(lim, dtype=object) #array of vertices
    global rad_search
    rad_search = r_search
    global obs
    obs = np.empty((0,2))
    global xy_graph
    xy_graph = np.zeros((0, 2))
    global resolution
    global shift_x
    global shift_y

    if calc_new_graph:
        time_calc_graph = calc_graph(world=world, start_xyp=start_xyp, save_file=save_file)
    else:
        #init_world
        #path to worlds dir
        path = os.path.dirname(os.path.abspath(__file__))
        path = os.path.dirname(path)
        #read yaml file
        add_to_path = 'worlds/' + world + '/' + world + '.yaml'
        yaml_path = os.path.join(path, add_to_path)
        with open(yaml_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
        origin = [data_loaded["origin"][0], data_loaded["origin"][1]]
        resolution = data_loaded["resolution"]
        shift_x = abs(origin[0]/resolution)
        shift_y = abs(origin[1]/resolution)
        #read pgm file
        add_to_path = 'worlds/' + world + '/' + world + '.pgm'
        init_path = os.path.join(path, add_to_path)
        init_world(init_path) #extract obs
        #load graph
        if load_saved_graph:
            add_to_path = 'worlds/' + world + '/graph_' + world + '.obj'
            load_path = os.path.join(path, add_to_path)
        else:
            load_path = os.path.join(path, 'worlds/graph.obj')
        file = open(load_path, "r") #open the file
        graph = pickle.load(file) #load the graph
        lim = len(graph)
        
        if len(obs) == 0:
            print("initialization of the world failed")
            print("choose between world1, world2, world3, world4, world5 and world6")
            return None, None, None, None, None
    
    if goal_xyp != None:
        trace, time_calc_trace = calc_trace(start_xyp, goal_xyp)
        
        if trace != None:
            print("found trace")
            print("trace: ")
            print_trace(trace)
            plot_it(trace)
            return True, trace, time_calc_graph, time_calc_trace
        else:
            print("found no trace")
            return False, trace, time_calc_graph, time_calc_trace
    
    else:
        return True, None, time_calc_graph, time_calc_trace



####################################################
#           launch main
####################################################  

world="world7" #selected world
start_xyp = (-4.56, 9.94, 2.1) #start position
goal_xyp = None#(15.0, 10.5, -0.4) #goal position
n_vertex = 3000 #count of nodes in the graph
r_search = 4 #search radius for find_neighbors
calc_new_graph = True #clac new graph or load a previously calced graph
load_saved_graph = True #load the last calced graph or the saved graph

main(world, start_xyp, goal_xyp, n_vertex, r_search, load_saved_graph, calc_new_graph)



'''
world1:
    Start: (0, 0, 1.57), Ziel: (4, -1, 0)
    trace: [(0, 0, 1.57), (1.83, 1.2, -0.48), (2.33, 0.1, -1.79), (3.07, -1.04, 0.02), (4, -1, 0)]

world2:
    Start: (0, 0, 0), Ziel: (3.75, -1.4, 0)
    trace: [(0, 0, 0), (-0.03, 0.035, 0.17), (0.74, 0.66, 1.07), (1.58, 1.02, -0.18), (2.6, -0.34, -1.31), (2.9, -0.95, -1.12), (3.75, -1.4, 0)]

world3:
    Start: (0, 0, -1.57), Ziel: (-1.75, -4, 3)
    trace: [(0, 0, -1.57), (-0.2, -0.45, -2.4), (-0.61, -2.42, -1.19), (-0.68, -3.4175, -2.01), (-1.75, -4, 3)]

world4:
    Start: (0, 0, 0), Ziel: (10, -0.5, 0)
    trace: [(0, 0, 0), (2.24, -0.04, 0.06), (5.29, -0.12, 0.01), (6.57, -0.24, -0.23), (10, -0.5, 0)]
    
world5:
    Start: (0, 0, 0), Ziel: (2.0, -3.7, 3.14)
    trace: [(0, 0, 0), (3.1, -0.45, -0,785), (2.0, -3.7, 3.14), (2.2, -1.8, 0.0)]

world6:
    Start: (0, 0, -0.4), Ziel: (15.0, 10.5, -0.4)
    trace: [(0, 0, -0.4), (1.5, -0.52, -0.2), (3.98, 0.66, 1.29), (4.98, 2.54, 1.05), (6.4, 5.92, 1.22), (8.14, 8.74, 0.95), (8.75, 10.49, 1.37), (12.33, 11.51, -0.3), (15.0, 10.5, -0.4)]
   
world7:
    Start: (-4.56, 9.94, 2.1), Ziel: (-5.24, 10.16, 2.1)
    trace: [(-4.56, 9.94, 2.1), (-14.6, 26.9, 2.1), (-6.7, 31.1, -1.04), (2.5, 14.5, -1.04), (-5.24, 10.16, 2.1)]
   
'''
    