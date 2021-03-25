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
def plot_arrow(xyp, color):
    arrow_length = 0.2
    dx = arrow_length * np.cos(xyp[2])
    dy = arrow_length * np.sin(xyp[2])
    plt.arrow( xyp[0], xyp[1], dx, dy, width = 0.01, color = color )



####################################################
#           main
####################################################  

goal = (0, 0, 0)

for x in range(-5, 5, 1):
    x = x/1.0
    print(x)
    for y in range(-5, 5, 1):
        y = y/1.0
        for phi in [-1.57, 0, 1.57, 3.14]:
            xyp = (x, y, phi)
            d = dist(xyp, goal)
            if d < 2:
                plot_arrow(xyp, 'blue')
            elif d < 4:
                plot_arrow(xyp, 'green')
            elif d < 6:
                plot_arrow(xyp, 'yellow')
            elif d < 8:
                plot_arrow(xyp, 'orange')
            else:
                plot_arrow(xyp, 'red')
    
    
plot_arrow(goal, 'red')

plt.xlim([-5, 5])
plt.ylim([-5, 5])

plt.show()























