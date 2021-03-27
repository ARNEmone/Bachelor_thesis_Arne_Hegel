import rosbag
import numpy as np
import matplotlib.pyplot as plt
import rospy
from PIL import Image
import random
import time 
import sys
import pickle
import resource
import os
import yaml

####################################################
#           Global Parameters
####################################################  

obs = np.empty((0,2)) #array of coordinates of obs

resolution = 0.05 #map param to load pgm
shift_x = 200 #map param to load pgm
shift_y = 200 #map param to load pgm

world = 'world7' #world of the rosbag i want to analyse (only needed for obs in odom)
to_analyse = 'rosbags/world7_3rounds_k_delta_2.bag' #rosbag i want to analyse



##############################################################
#           functions
##############################################################

#plots an arrow out of xyp
def plot_arrow(xyp, color):
    arrow_length = 0.1
    dx = arrow_length * np.cos(xyp[2])
    dy = arrow_length * np.sin(xyp[2])
    ax[0,1].arrow( xyp[0], xyp[1], dx, dy, width = 0.01, color = color)
    
#plots an arrow out of xyp
def plot_arrow_plt(xyp, color):
    arrow_length = 0.3
    dx = arrow_length * np.cos(xyp[2])
    dy = arrow_length * np.sin(xyp[2])
    plt.arrow( xyp[0], xyp[1], dx, dy, width = 0.05, color = color)

#returns true if xyp1 is near xyp2
def near_to(xyp1, xyp2, rad):
    if abs(xyp1[0] - xyp2[0]) < rad and abs(xyp1[1] - xyp2[1]) < rad:
        return True
    else:
        return False

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

def init_w(world):
    global resolution
    global shift_x
    global shift_y
    
    if world == None:
        return None
    
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

#map/graph plot
def plot_obs_ax():
    ax[0,1].axis('equal')
    x1 = []
    y1 = []
    for o in obs:
        x1.append(o[0])
        y1.append(o[1])
    ax[0,1].plot(x1, y1, 'ks', label='Hindernis')
    
#map/graph plot
def plot_obs_plt():
    plt.axis('equal')
    x1 = []
    y1 = []
    for o in obs:
        x1.append(o[0])
        y1.append(o[1])
    plt.plot(x1, y1, 'ks', label='Hindernis')



##############################################################
#           get data of rosbag
##############################################################

bag = rosbag.Bag(to_analyse) #loads rosbag

#dist
dist = []
dist_time=[]
for topic, msg, t in bag.read_messages(topics=['/dist_2_goal_throttled']):
    dist.append(msg.data)
    dist_time.append(rospy.Time.to_sec(t))

'''
#partition_dist
partition_dist = []
for topic, msg, t in bag.read_messages(topics=['/partition_dist_2_goal']):
    partition_dist.append(msg.data)
'''

#theta
theta = []
theta_time=[]
for topic, msg, t in bag.read_messages(topics=['/theta_throttled']):
    theta.append(msg.data)
    theta_time.append(rospy.Time.to_sec(t))
    
#odom
odom_x = []
odom_y = []
odom_time = []
for topic, msg, t in bag.read_messages(topics=['/odom_throttled']):
    odom_x.append(msg.pose.pose.position.x)
    odom_y.append(msg.pose.pose.position.y)
    odom_time.append(rospy.Time.to_sec(t))

#trace
trace = []
get_trace = []
for topic, msg, t in bag.read_messages(topics=['/trace_throttled']):
    get_trace = msg.data
for i in range(0, len(get_trace), 3):
    trace.append([get_trace[i], get_trace[i+1], get_trace[i+2]])

print("trace: ", trace)

#i_goal
i_goal = []
i_goal_time = []
partition_goal = []
for topic, msg, t in bag.read_messages(topics=['/partition_goal_xyp_throttled']):
    partition_goal.append(msg.data)
    i_goal_time.append(rospy.Time.to_sec(t))
i = 0
i_goal.append(0)
for i_pg in range(1, len(partition_goal), 1):
    if partition_goal[i_pg] != partition_goal[i_pg-1]:
        i += 1
    i_goal.append(i)

bag.close()



##############################################################
#           cut
##############################################################



pg_start = (0.5,-6.5,1.17)
pg_stop = (0.5,-6.5,1.17)

#returns true if xyp1 is near xyp2
def near_to(xyp1, xyp2, rad):
    if abs(xyp1[0] - xyp2[0]) < rad and abs(xyp1[1] - xyp2[1]) < rad:
        return True
    else:
        return False

#world7_3rounds_k_delta_1
#1 round
#time_start = 1616856317
#time_stop = 1616856437
#2 round
#time_start = 1616856437
#time_stop = 1616856566
#3 round
#time_start = 1616856566
#time_stop = 1616856676

#world7_3rounds_k_delta_2
#1 round
#time_start = 1616859382
#time_stop = 1616859502
#2 round
#time_start = 1616859502
#time_stop = 1616859631
#3 round
time_start = 1616859631
time_stop = 1616859742

#world7_3rounds_k_delta_3
#1 round
#time_start = 1616857674
#time_stop = 1616857794
#2 round
#time_start = 1616857794
#time_stop = 1616857923
#3 round
#time_start = 1616857923
#time_stop = 1616858034

#dist
for i in range(len(dist)):
    if dist_time[i] - time_start > 0.1:
        dist = dist[i:]
        dist_time = dist_time[i:]
        break
for i in range(len(dist)):
    if dist_time[i] - time_stop > 0.1:
        dist = dist[:i]
        dist_time = dist_time[:i]
        break

#theta
for i in range(len(theta)):
    if theta_time[i] - time_start > 0.1:
        theta = theta[i:]
        theta_time = theta_time[i:]
        break
for i in range(len(theta)):
    if theta_time[i] - time_stop > 0.1:
        theta = theta[:i]
        theta_time = theta_time[:i]
        break

#theta
for i in range(len(i_goal)):
    if i_goal_time[i] - time_start > 0.1:
        i_goal = i_goal[i:]
        i_goal_time = i_goal_time[i:]
        break
for i in range(len(i_goal)):
    if i_goal_time[i] - time_stop > 0.1:
        i_goal = i_goal[:i]
        i_goal_time = i_goal_time[:i]
        break

#odom
for i in range(len(odom_time)):
    if odom_time[i] - time_start > 0.1:
        odom_x = odom_x[i:]
        odom_y = odom_y[i:]
        odom_time = odom_time[i:]
        break
for i in range(len(odom_time)):
    if odom_time[i] - time_stop > 0.1:
        odom_x = odom_x[:i]
        odom_y = odom_y[:i]
        odom_time = odom_time[:i]
        break


print("start: ", i_goal_time[0], "stop: ", i_goal_time[len(i_goal_time)-1])


##############################################################
#           evaluation (marks inadequatly driven sections)
##############################################################

#dist
dist_badly_driven = [] #array with badly dirven sections
dist_badly_driven_time = [] #time array with badly dirven sections
index = 50 #index
while index < len(dist):
    badly_driven_section = []
    badly_driven_section_time = []
    while index < len(dist):
        dist_grows = False #ture if all last three dist are greater than the selected
        for i in range(index-50, index, 5):
            if dist[i] < dist[index]:
                dist_grows = True
        if dist_grows:
            badly_driven_section.append(dist[index])
            badly_driven_section_time.append(dist_time[index])
        else:
            break
        index += 1
    if len(badly_driven_section) > 0:
        if (badly_driven_section_time[len(badly_driven_section_time)-1] - badly_driven_section_time[0]) > 0.5:
            #print(badly_driven_section_time[len(badly_driven_section_time)-1] - badly_driven_section_time[0])
            dist_badly_driven.append(badly_driven_section)
            dist_badly_driven_time.append(badly_driven_section_time)
    index += 1

#trace
x_badly_driven = [] #array with badly dirven sections
y_badly_driven = [] #array with badly dirven sections
index = 0 #index
for i in range(len(dist_badly_driven_time)):
    while index < (len(odom_time)-1):
        if (odom_time[index] - dist_badly_driven_time[i][0]) > -0.5:
            x_badly_driven_section = [] #array with badly dirven sections
            y_badly_driven_section = [] #array with badly dirven sections
            while index < (len(odom_time)-1) and (odom_time[index] - dist_badly_driven_time[i][len(dist_badly_driven_time[i])-1]) < 0.5:
                x_badly_driven_section.append(odom_x[index])
                y_badly_driven_section.append(odom_y[index])
                index += 1
            x_badly_driven.append(x_badly_driven_section)
            y_badly_driven.append(y_badly_driven_section)
            break
        index += 1


##############################################################
#           first pic with 4 plots
##############################################################

fig, ax = plt.subplots(2, 2)

#dist
ax[1,1].plot(dist_time, dist, 'y-', label="gut gefahren")

for i in range(len(dist_badly_driven)):
    ax[1,1].plot(dist_badly_driven_time[i], dist_badly_driven[i], 'k-', label="inadaequat gefahren")

#vertical lines
for i in range(1, len(i_goal), 1):
    if i_goal[i-1] < i_goal[i]:
        ax[1,1].axvline(x=i_goal_time[i], label='betrachtetes Zwischenziel wechselt')

ax[1,1].set_title('Distanz zum Ziel')
ax[1,1].set_xlabel('Zeit [s]')
ax[1,1].set_ylabel('Distanz')
ax[1,1].legend()

#############################

#i_goal
ax[1,0].plot(i_goal_time, i_goal)
ax[1,0].set_title('Zielindex')
ax[1,0].set_xlabel('Zeit [s]')
ax[1,0].set_ylabel('Zielindex')

#############################

#trace
init_w(world)
plot_obs_ax()

ax[0,1].plot(odom_x, odom_y, 'y-', label="gefahrener Weg") #plot trace

for i in range(len(x_badly_driven)): #plot badly driven trace
    ax[0,1].plot(x_badly_driven[i], y_badly_driven[i], 'k-', label="inadaequat gefahren")

for i in range(1, len(trace)-1, 1): #plot interim goals
    plot_arrow(trace[i], 'b')
    ax[0,1].plot(trace[i][0], trace[i][1], 'bo', label="Zwischenziel")

plot_arrow(trace[0], 'k') #plot start
ax[0,1].plot(trace[0][0], trace[0][1], 'ko', label="Start") 

plot_arrow(trace[len(trace)-1], 'r') #plot goal
ax[0,1].plot(trace[len(trace)-1][0], trace[len(trace)-1][1], 'ro', label="Ziel")

ax[0,1].set_title('Fahrt')
ax[0,1].set_xlabel('x [m]')
ax[0,1].set_ylabel('y [m]')
ax[0,1].legend()

#############################

#theta
ax[0,0].plot(theta_time, theta)
ax[0,0].set_title('theta')
ax[0,0].set_xlabel('Zeit [s]')
ax[0,0].set_ylabel('theta')

plt.show()



##############################################################
#           second pic (trace)
##############################################################

plot_obs_plt()

plt.plot(odom_x, odom_y, 'y-', label="gefahrener Weg") #plot trace

for i in range(len(x_badly_driven)): #plot badly driven trace
    plt.plot(x_badly_driven[i], y_badly_driven[i], 'k-', label="inadaequat gefahren")

for i in range(1, len(trace)-1, 1): #plot interim goals
    plot_arrow_plt(trace[i], 'b')
    plt.plot(trace[i][0], trace[i][1], 'bo', markersize=2.0, label="Zwischenziel")

plot_arrow_plt(trace[0], 'k') #plot start
plt.plot(trace[0][0], trace[0][1], 'ko', markersize=2.0, label="Start") 

plot_arrow_plt(trace[len(trace)-1], 'r') #plot goal
plt.plot(trace[len(trace)-1][0], trace[len(trace)-1][1], 'ro', markersize=2.0, label="Ziel")

plt.title('Fahrt')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()

plt.show()


##############################################################
#           third pic (dist)
##############################################################

plt.plot(dist_time, dist, 'y-', label="gut gefahren")

for i in range(len(dist_badly_driven)):
    plt.plot(dist_badly_driven_time[i], dist_badly_driven[i], 'k-', label="inadaequat gefahren")

#vertical lines
for i in range(1, len(i_goal), 1):
    if i_goal[i-1] < i_goal[i]:
        plt.axvline(x=i_goal_time[i], label='betrachtetes Zwischenziel wechselt')

plt.title('Distanz zum Ziel')
plt.xlabel('Zeit [s]')
plt.ylabel('Distanz')
plt.legend()


plt.show()
