import numpy as np
import math
import matplotlib.pyplot as plt

####################################################
#           Parameters
####################################################  

k_phi = 1 #positive constant that represent the weight of phi with respect to r.
k_delta = 1 #positive constant that represent the weight of delta with respect to phi.



####################################################
#           help functions
####################################################  

def wrap_to_pi(angle):
	wrappedAngle = angle %  math.pi
	times = angle//math.pi
	if (times % 2) == 1: # ungerade
		wrappedAngle = wrappedAngle - math.pi 
	return wrappedAngle

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

#cost function
def dist(xyp_from, xyp_to):
    rpd = convert_xyp_2_rpd(xyp_from, xyp_to)
    l_minus = np.sqrt((rpd[0]**2) + (k_phi**2) * (rpd[1]**2))
    delta_e = abs( rpd[2] - math.atan(-1 * k_phi * rpd[1]) )
    return l_minus + k_delta * delta_e

#plots an arrow out of xyp
def plot_arrow(xyp, color, label):
    arrow_length = 0.1
    dx = arrow_length * np.cos(xyp[2])
    dy = arrow_length * np.sin(xyp[2])
    plt.arrow( xyp[0], xyp[1], dx, dy, width = 0.01, color = color, label='dist<1' )



####################################################
#           main
####################################################  

goal = (0, 0, 0)

for x in range(-20, 20, 1):
    x = x/4.0
    print(x)
    for y in range(-20, 20, 1):
        y = y/4.0
        d_min = 1000
        phi_min = 1000
        for phi in range(-310, 310, 1):
            phi = phi/100.0
            xyp = (x, y, phi)
            d = dist(xyp, goal)
            if d < d_min:
                d_min = d
                phi_min = phi
        if d_min < 1:
            plot_arrow((x, y, phi_min), 'blue', label='dist<1')
        elif d_min < 2:
            plot_arrow((x, y, phi_min), 'green', label='dist<2')
        elif d_min < 3:
            plot_arrow((x, y, phi_min), 'yellow', label='dist<3')
        elif d_min < 4:
            plot_arrow((x, y, phi_min), 'orange', label='dist<4')
        else:
            plot_arrow((x, y, phi_min), 'red', label='dist>4')
    
    
plot_arrow(goal, 'red', 'goal')

plt.xlim([-2.5, 2.5])
plt.ylim([-2.5, 2.5])

plt.xlabel('x')
plt.ylabel('y')


plt.show()























