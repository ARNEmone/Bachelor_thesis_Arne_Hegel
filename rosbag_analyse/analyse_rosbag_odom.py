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
#           GAME Parameters
####################################################  

obs = np.empty((0,2)) #array of coordinates of obs

resolution = 0.05 #map param to load pgm
shift_x = 200 #map param to load pgm
shift_y = 200 #map param to load pgm

world = "world7" #world of the rosbag i want to analyse
to_analyse = 'rosbags/world7_odom.bag' #rosbag i want to analyse



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

def init_world(to_open):
    #load img
    img = Image.open(to_open)
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
    
#odom
odom_x = []
odom_y = []
odom_time = []
for topic, msg, t in bag.read_messages(topics=['/odom']):
    odom_x.append(msg.pose.pose.position.x)
    odom_y.append(msg.pose.pose.position.y)
    odom_time.append(rospy.Time.to_sec(t))

bag.close()



##############################################################
#           cut
##############################################################

"""

pg_start = (0.5,-6.5,1.17)
pg_stop = (0.5,-6.5,1.17)

#returns true if xyp1 is near xyp2
def near_to(xyp1, xyp2, rad):
    if abs(xyp1[0] - xyp2[0]) < rad and abs(xyp1[1] - xyp2[1]) < rad:
        return True
    else:
        return False

#analyse_world6_1newnewTheta
#1 round
#time_start = 1615819599
#time_stop = 1615819760
#2 round
#time_start = 1615819760
#time_stop = 1615819880
#3 round
#time_start = 1615819880
#time_stop = 1615819990

#analyse_world6_1newnewTheta_5000_6_k_delta_1
#1 round
#time_start = 1615998370
#time_stop = 1615998500
#2 round
#time_start = 1615998500
#time_stop = 1615998620
#3 round
#time_start = 1615998620
#time_stop = 1615998723

#analyse_world6_1newnewTheta_5000_6_k_delta_3
#1 round
#time_start = 1616065370
#time_stop = 1616065500
#2 round
#time_start = 1616065500
#time_stop = 1616065620
#3 round
#time_start = 1616065620
#time_stop = 1616065722

#analyse_world6_1newnewTheta_myGoals_k_delta_3
#1 round
#time_start = 1616068050
#time_stop = 1616068180
#2 round
#time_start = 1616068180
#time_stop = 1616068300
#3 round
#time_start = 1616068300
#time_stop = 1616068402

#analyse_world6_1newnewTheta_myGoals_k_delta_5
#1 round
#time_start = 1616069289
#time_stop = 1616069419
#2 round
#time_start = 1616069419
#time_stop = 1616069539
#3 round
#time_start = 1616069539
#time_stop = 1616069641

#analyse_world6_1newnewTheta_myGoals_k_delta_5
#1 round
#time_start = 1616069289
#time_stop = 1616069419
#2 round
#time_start = 1616069419
#time_stop = 1616069539
#3 round
#time_start = 1616069539
#time_stop = 1616069641

#analyse_world6_2newnewTheta_5000_6_k_delta_1
#1 round
time_start = 1616074190
time_stop = 1616074310
#2 round
#time_start = 1616074320
#time_stop = 1616074440
#3 round
#time_start = 1616074440
#time_stop = 1616074542

#analyse_world6_2newnewTheta_5000_6_k_delta_3
#1 round
#time_start = 1616075219
#time_stop = 1616075349
#2 round
#time_start = 1616075349
#time_stop = 1616075469
#3 round
#time_start = 1616075469
#time_stop = 1616075571

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

"""
print("start_time: ", odom_time[0], "stop_time: ", odom_time[len(odom_time)-1])
print("start_pose: ", odom_x[0], odom_y[0], "stop_pose: ", odom_x[len(odom_time)-1], odom_y[len(odom_time)-1])


##############################################################
#           second pic (trace)
##############################################################

init_w(world)

plot_obs_plt()

plt.plot(odom_x, odom_y, 'y-', label="gefahrener Weg") #plot trace

plt.plot(odom_x[0], odom_y[0], 'bo', label="Start") #plot start
plt.plot(odom_x[len(odom_x)-1], odom_y[len(odom_y)-1], 'ro', label="Ziel") #plot goal

plt.title('Fahrt')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()

plt.show()


