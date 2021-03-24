#! /usr/bin/env python
import numpy as np
import rospy
import sys
import os
import math

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32

from RRT_star_graph_and_a_star import main, vertex, dist


####################################################
#           GAME Parameters
####################################################  

k_phi = 1.2 #positive constant that represent the weight of phi with respect to r.
k_delta = 1 #positive constant that represent the weight of delta with respect to phi.
shift_distanz = 0.2 #dist how much to shift coordinates in shift_x0_to_x1
transition_parameter = 2 #the higher the harder the transition in the middle; 1 = linear



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

#returns true if xy1 is near xy2
def near_to(xy1, xy2, rad):
    if abs(xy1[0] - xy2[0]) < rad and abs(xy1[1] - xy2[1]) < rad:
        return True
    return False

#euclidean distance
def eucl_dist(coord1, coord2):
    delta_x = coord2[0] - coord1[0]
    delta_y = coord2[1] - coord1[1]
    return np.sqrt(delta_x**2 + delta_y**2)



####################################################
#           theta functions
####################################################  

#theta on [0,1] to describe the transition between xy0 and xy1
#odom=xy0 -> theta=0; odom=xy1 -> theta=1 
def calc_theta(x, x0, x1):
    x0_new = shift_x0_to_x1(x0, x1)
    x1_new = shift_x0_to_x1(x1, x0)
    a = eucl_dist(x[:2], x1_new)
    b = eucl_dist(x0_new, x[:2])
    c = eucl_dist(x0_new, x1_new)
    s = 0.5*(a+b+c)
    h_c = (2/c)*np.sqrt(s*(s-a)*(s-b)*(s-c))
    c_1 = np.sqrt(b**2 - h_c**2)
    c_2 = c - c_1
    
    return c_1**2/(c_2**2+c_1**2)

#calcs angle between x0-x and x0-x1 and returns true if angle <= 1.57
def start_theta(x, x0, x1):
    x = np.array(x[:2])
    x0 = shift_x0_to_x1(x0, x1)
    x1 = np.array(x1[:2])
    a = x0-x #vector 1
    b = x0-x1 #vector 2
    
    dot = np.dot(a,b) #skalarprodukt
    a_modulus = np.sqrt((a*a).sum())
    b_modulus = np.sqrt((b*b).sum())
    cos_angle = dot / a_modulus / b_modulus 
    if cos_angle > 1:
        angle = 0
    elif cos_angle < -1:
        angle = np.pi
    else:
        angle = np.arccos(cos_angle) # Winkel in Bogenmass
    
    return angle <= 1.57

#calcs angle between x1-x and x1-x0 and returns true if angle >= 1.57
def stop_theta(x, x0, x1):
    if eucl_dist(x0, x1) < (shift_distanz+0.1):
        return True

    x = np.array(x[:2])
    x1 = shift_x0_to_x1(x1, x0)
    x0 = np.array(x0[:2])
    a = x1-x #vector 1
    b = x1-x0 #vector 2
    
    dot = np.dot(a,b) #skalarprodukt
    a_modulus = np.sqrt((a*a).sum())
    b_modulus = np.sqrt((b*b).sum())
    cos_angle = dot / a_modulus / b_modulus
    if cos_angle > 1:
        angle = 0
    elif cos_angle < -1:
        angle = np.pi
    else:
        angle = np.arccos(cos_angle) # Winkel in Bogenmass
    
    return angle >= 1.57

#shift x1 in direction of x1 
def shift_x0_to_x1(x0, x1):
    if x0[:2] == x1[:2]:
        return x0
    
    delta_x = x1[0] - x0[0]
    delta_y = x1[1] - x0[1]
    angle = math.atan2(delta_y, delta_x) # angle of line-of-sight
    
    return (x0[0]+shift_distanz*np.cos(angle), x0[1]+shift_distanz*np.sin(angle) )
    


####################################################
#           class
####################################################  

class TargetCalculation:

    def __init__(self):
        
        #goal array
        #split goal_array string
        self.goal_array = sys.argv[6]
        self.goal_array = self.goal_array.split(';')
        for i in range(len(self.goal_array)):
            self.goal_array[i] = self.goal_array[i].split(',')
        #convert to float
        for i in range(len(self.goal_array)):
            for j in range(len(self.goal_array[i])):
                self.goal_array[i][j] = float(self.goal_array[i][j])
        
        #my odom xy
        self.odom_xyp = [float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]
        
        #init goal
        self.theta = Float32()
        
        #init partition goal
        self.partition_goal_xyp = Float32MultiArray()
        
        #subscriber and publisher
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        pub_theta = rospy.Publisher("theta", Float32, queue_size=10)
        pub_partition_goal_xyp = rospy.Publisher("partition_goal_xyp", Float32MultiArray, queue_size=10)
        pub_trace = rospy.Publisher("trace", Float32MultiArray, queue_size=10)
        pub_dist = rospy.Publisher("dist_2_goal", Float32, queue_size=10)
        pub_partition_dist = rospy.Publisher("partition_dist_2_goal", Float32, queue_size=10)
        
        ################################################
        #init used variables
        
        i_goal = 1 #index of trace goal; starting with 1 because first partition_goal ist the start xyp
        next_found_trace = False
        trace = [] #trace from odom 2  i_goal
        partition_costs = [] #all costs between the partition goals in trace
        trace_pub_array = []
        
        #################################################
        #get trace and costs
        
        trace.append([float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])])
        #calc traces between goals
        for i in range(len(self.goal_array)-1):
            while not next_found_trace:
                next_found_trace, next_trace, next_cost, next_partition_costs = main(sys.argv[1], self.goal_array[i], self.goal_array[i+1], sys.argv[2], False)
            for i_trace in range(1,len(next_trace),1):
                trace.append(next_trace[i_trace])
            next_found_trace = False
        
        #world5
        #trace = [(0.0, 0.0, 0.0), (1.29, 0.41, 0.3), (3.1, -0.45, -0.785), (2.0, -3.7, 3.14), (0.87, -2.01, 0.54), (2.24, -1.72, -0.02), (2.2, -1.8, 0.0)]
        
        #world6_1
        #trace = [[0.0, 0.0, -0.4], [1.63, -0.46, 0.059], [3.42, -0.259, 1.09], [4.69, 1.45, 0.85], [6.269, 3.93, 0.89], [7.809, 6.929, 1.12], [8.88, 9.010000228881836, 0.949999988079071], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [10.640000343322754, 11.720000267028809, 0.6899999976158142], [12.6899995803833, 11.350000381469727, -0.4399999976158142], [14.5, 11.0600004196167, 0.3199999928474426], [15.0, 11.0, -0.4000000059604645], [16.93000030517578, 9.84000015258789, -0.800000011920929], [17.540000915527344, 7.510000228881836, -1.4900000095367432], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.670000076293945, 4.599999904632568, -2.0999999046325684], [14.850000381469727, 1.1799999475479126, -2.049999952316284], [13.600000381469727, -2.009999990463257, -1.8600000143051147], [11.630000114440918, -4.820000171661377, -2.25], [10.210000038146973, -7.28000020980835, -1.9800000190734863], [9.399999618530273, -8.9399995803833, -1.840000033378601], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.809999942779541, -12.109999656677246, -2.1500000953674316], [4.590000152587891, -12.600000381469727, 2.7699999809265137], [2.9000000953674316, -11.800000190734863, 2.740000009536743], [0.9100000262260437, -9.9399995803833, 1.9900000095367432], [0.28999999165534973, -7.570000171661377, 1.399999976158142], [0.5, -6.5, 1.1699999570846558], [1.0466127395629883, -5.010176181793213, 1.0299999713897705], [2.1500000953674316, -2.5299999713897705, 1.1699999570846558], [3.130000114440918, -0.6399999856948853, 0.8999999761581421]]
        #world6_3
        #trace = [[0.0, 0.0, -0.4], [1.63, -0.46, 0.059], [3.42, -0.259, 1.09], [4.69, 1.45, 0.85], [6.269, 3.93, 0.89], [7.809, 6.929, 1.12], [8.88, 9.010000228881836, 0.949999988079071], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [10.640000343322754, 11.720000267028809, 0.6899999976158142], [12.6899995803833, 11.350000381469727, -0.4399999976158142], [14.5, 11.0600004196167, 0.3199999928474426], [15.0, 11.0, -0.4000000059604645], [16.93000030517578, 9.84000015258789, -0.800000011920929], [17.540000915527344, 7.510000228881836, -1.4900000095367432], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.670000076293945, 4.599999904632568, -2.0999999046325684], [14.850000381469727, 1.1799999475479126, -2.049999952316284], [13.600000381469727, -2.009999990463257, -1.8600000143051147], [11.630000114440918, -4.820000171661377, -2.25], [10.210000038146973, -7.28000020980835, -1.9800000190734863], [9.399999618530273, -8.9399995803833, -1.840000033378601], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.809999942779541, -12.109999656677246, -2.1500000953674316], [4.590000152587891, -12.600000381469727, 2.7699999809265137], [2.9000000953674316, -11.800000190734863, 2.740000009536743], [0.9100000262260437, -9.9399995803833, 1.9900000095367432], [0.28999999165534973, -7.570000171661377, 1.399999976158142], [0.5, -6.5, 1.1699999570846558], [1.0466127395629883, -5.010176181793213, 1.0299999713897705], [2.1500000953674316, -2.5299999713897705, 1.1699999570846558], [3.130000114440918, -0.6399999856948853, 0.8999999761581421], [4.75, 1.909999966621399, 1.0800000429153442], [6.269999980926514, 3.930000066757202, 0.8999999761581421], [7.809999942779541, 6.929999828338623, 1.1200000047683716], [8.880000114440918, 9.010000228881836, 0.949999988079071], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [10.640000343322754, 11.720000267028809, 0.6899999976158142], [12.6899995803833, 11.350000381469727, -0.4399999976158142], [14.5, 11.0600004196167, 0.3199999928474426], [15.0, 11.0, -0.4000000059604645], [16.93000030517578, 9.84000015258789, -0.800000011920929], [17.540000915527344, 7.510000228881836, -1.4900000095367432], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.670000076293945, 4.599999904632568, -2.0999999046325684], [14.850000381469727, 1.1799999475479126, -2.049999952316284], [13.600000381469727, -2.009999990463257, -1.8600000143051147], [11.630000114440918, -4.820000171661377, -2.25], [10.210000038146973, -7.28000020980835, -1.9800000190734863], [9.399999618530273, -8.9399995803833, -1.840000033378601], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.809999942779541, -12.109999656677246, -2.1500000953674316], [4.590000152587891, -12.600000381469727, 2.7699999809265137], [2.9000000953674316, -11.800000190734863, 2.740000009536743], [0.9100000262260437, -9.9399995803833, 1.9900000095367432], [0.28999999165534973, -7.570000171661377, 1.399999976158142], [0.5, -6.5, 1.1699999570846558], [1.0466127395629883, -5.010176181793213, 1.0299999713897705], [2.1500000953674316, -2.5299999713897705, 1.1699999570846558], [3.130000114440918, -0.6399999856948853, 0.8999999761581421], [4.75, 1.909999966621399, 1.0800000429153442], [6.269999980926514, 3.930000066757202, 0.8999999761581421], [7.809999942779541, 6.929999828338623, 1.1200000047683716], [8.880000114440918, 9.010000228881836, 0.949999988079071], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [10.640000343322754, 11.720000267028809, 0.6899999976158142], [12.6899995803833, 11.350000381469727, -0.4399999976158142], [14.5, 11.0600004196167, 0.3199999928474426], [15.0, 11.0, -0.4000000059604645], [16.93000030517578, 9.84000015258789, -0.800000011920929], [17.540000915527344, 7.510000228881836, -1.4900000095367432], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.670000076293945, 4.599999904632568, -2.0999999046325684], [14.850000381469727, 1.1799999475479126, -2.049999952316284], [13.600000381469727, -2.009999990463257, -1.8600000143051147], [11.630000114440918, -4.820000171661377, -2.25], [10.210000038146973, -7.28000020980835, -1.9800000190734863], [9.399999618530273, -8.9399995803833, -1.840000033378601], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.809999942779541, -12.109999656677246, -2.1500000953674316], [4.590000152587891, -12.600000381469727, 2.7699999809265137], [2.9000000953674316, -11.800000190734863, 2.740000009536743], [0.9100000262260437, -9.9399995803833, 1.9900000095367432], [0.28999999165534973, -7.570000171661377, 1.399999976158142], [0.5, -6.5, 1.1699999570846558], [1.0466127395629883, -5.010176181793213, 1.0299999713897705], [1.2699999809265137, -4.829999923706055, 0.5400000214576721], [2.2699999809265137, -1.2999999523162842, 2.049999952316284], [0.75, 0.07999999821186066, 2.5199999809265137], [0.20000000298023224, 0.4000000059604645, 2.740000009536743]]
        #world6_3 v2
        #trace = [[0.0, 0.0, -0.4000000059604645], [0.3100000023841858, -0.11999999731779099, -0.07999999821186066], [4.670441150665283, 0.8242916464805603, 1.3899999856948853], [6.159999847412109, 5.329999923706055, 0.8700000047683716], [8.4399995803833, 8.579999923706055, 0.800000011920929], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [11.039999961853027, 11.539999961853027, 0.6000000238418579], [16.489999771118164, 10.5, -0.9200000166893005], [17.309999465942383, 6.03000020980835, -2.049999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.309999465942383, 4.03000020980835, -2.140000104904175], [13.760000228881836, -0.7200000286102295, -1.9700000286102295], [11.789999961853027, -4.929999828338623, -2.0899999141693115], [9.390000343322754, -9.460000038146973, -2.3499999046325684], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.28000020980835, -12.760000228881836, -2.609999895095825], [5.340000152587891, -13.09000015258789, 3.069999933242798], [0.6700000166893005, -9.720000267028809, 1.9500000476837158], [0.25999999046325684, -7.079999923706055, 1.2999999523162842], [0.5, -6.5, 1.1699999570846558], [0.3100000023841858, -0.11999999731779099, -0.07999999821186066], [4.670441150665283, 0.8242916464805603, 1.3899999856948853], [6.159999847412109, 5.329999923706055, 0.8700000047683716], [8.4399995803833, 8.579999923706055, 0.800000011920929], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [11.039999961853027, 11.539999961853027, 0.6000000238418579], [16.489999771118164, 10.5, -0.9200000166893005], [17.309999465942383, 6.03000020980835, -2.049999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.309999465942383, 4.03000020980835, -2.140000104904175], [13.760000228881836, -0.7200000286102295, -1.9700000286102295], [11.789999961853027, -4.929999828338623, -2.0899999141693115], [9.390000343322754, -9.460000038146973, -2.3499999046325684], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.28000020980835, -12.760000228881836, -2.609999895095825], [5.340000152587891, -13.09000015258789, 3.069999933242798], [0.6700000166893005, -9.720000267028809, 1.9500000476837158], [0.25999999046325684, -7.079999923706055, 1.2999999523162842], [0.5, -6.5, 1.1699999570846558], [0.3100000023841858, -0.11999999731779099, -0.07999999821186066], [4.670441150665283, 0.8242916464805603, 1.3899999856948853], [6.159999847412109, 5.329999923706055, 0.8700000047683716], [8.4399995803833, 8.579999923706055, 0.800000011920929], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [11.039999961853027, 11.539999961853027, 0.6000000238418579], [16.489999771118164, 10.5, -0.9200000166893005], [17.309999465942383, 6.03000020980835, -2.049999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [16.309999465942383, 4.03000020980835, -2.140000104904175], [13.760000228881836, -0.7200000286102295, -1.9700000286102295], [11.789999961853027, -4.929999828338623, -2.0899999141693115], [9.390000343322754, -9.460000038146973, -2.3499999046325684], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [7.28000020980835, -12.760000228881836, -2.609999895095825], [5.340000152587891, -13.09000015258789, 3.069999933242798], [0.6700000166893005, -9.720000267028809, 1.9500000476837158], [0.25999999046325684, -7.079999923706055, 1.2999999523162842], [0.5, -6.5, 1.1699999570846558]]
        #world6_3 v3
        #trace = [[0.0, 0.0, -0.4000000059604645], [1.2799999713897705, -0.07999999821186066, 0.20000000298023224], [3.680000066757202, -0.27000001072883606, 0.949999988079071], [4.579999923706055, 0.8700000047683716, 0.7400000095367432], [5.039999961853027, 1.4199999570846558, 1.090000033378601], [7.119999885559082, 5.53000020980835, 1.1699999570846558], [8.180000305175781, 7.619999885559082, 0.8899999856948853], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [12.239999771118164, 11.569999694824219, -0.09000000357627869], [14.65999984741211, 11.010000228881836, -0.3799999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [15.404963493347168, 2.307802438735962, -2.0799999237060547], [13.170000076293945, -1.8700000047683716, -2.049999952316284], [10.970000267028809, -6.0, -2.1500000953674316], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [5.869999885559082, -12.880000114440918, 3.119999885559082], [0.9599999785423279, -10.020000457763672, 2.240000009536743], [0.5, -6.5, 1.1699999570846558], [1.2799999713897705, -0.07999999821186066, 0.20000000298023224], [3.680000066757202, -0.27000001072883606, 0.949999988079071], [4.579999923706055, 0.8700000047683716, 0.7400000095367432], [5.039999961853027, 1.4199999570846558, 1.090000033378601], [7.119999885559082, 5.53000020980835, 1.1699999570846558], [8.180000305175781, 7.619999885559082, 0.8899999856948853], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [12.239999771118164, 11.569999694824219, -0.09000000357627869], [14.65999984741211, 11.010000228881836, -0.3799999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [15.404963493347168, 2.307802438735962, -2.0799999237060547], [13.170000076293945, -1.8700000047683716, -2.049999952316284], [10.970000267028809, -6.0, -2.1500000953674316], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [5.869999885559082, -12.880000114440918, 3.119999885559082], [0.9599999785423279, -10.020000457763672, 2.240000009536743], [0.5, -6.5, 1.1699999570846558], [1.2799999713897705, -0.07999999821186066, 0.20000000298023224], [3.680000066757202, -0.27000001072883606, 0.949999988079071], [4.579999923706055, 0.8700000047683716, 0.7400000095367432], [5.039999961853027, 1.4199999570846558, 1.090000033378601], [7.119999885559082, 5.53000020980835, 1.1699999570846558], [8.180000305175781, 7.619999885559082, 0.8899999856948853], [9.300000190734863, 9.800000190734863, 1.1699999570846558], [12.239999771118164, 11.569999694824219, -0.09000000357627869], [14.65999984741211, 11.010000228881836, -0.3799999952316284], [17.200000762939453, 5.800000190734863, -1.9700000286102295], [15.404963493347168, 2.307802438735962, -2.0799999237060547], [13.170000076293945, -1.8700000047683716, -2.049999952316284], [10.970000267028809, -6.0, -2.1500000953674316], [8.600000381469727, -10.800000190734863, -1.9700000286102295], [5.869999885559082, -12.880000114440918, 3.119999885559082], [0.9599999785423279, -10.020000457763672, 2.240000009536743], [0.5, -6.5, 1.1699999570846558]]
        #world6_3 my pg
        #trace = [[0.0, 0.0, -0.4], [4.67, 0.824, 1.389], [7.5, 6.9, 1.17], [11.039, 11.539, 0.6], [14, 11.5, -0.4], [16.9, 9.7, -0.5], [17.2, 5.8, -1.97], [13.76, -0.72, -1.97], [10.1, -7.7, -1.97], [7.28, -12.76, -2.609], [2.0, -10.8, 2.74], [0.5, -6.5, 1.169], [4.67, 0.824, 1.389], [7.5, 6.9,1.17], [11.039, 11.539, 0.6], [14, 11.5, -0.4], [16.9, 9.7, -0.5], [17.2, 5.8, -1.97], [13.76, -0.72, -1.97], [10.1, -7.7, -1.97], [7.28, -12.76, -2.609], [2.0, -10.8, 2.74], [0.5, -6.5, 1.169], [4.67, 0.824, 1.389], [7.5, 6.9,1.17], [11.039, 11.539, 0.6], [14, 11.5, -0.4], [16.9, 9.7, -0.5], [17.2, 5.8, -1.97], [13.76, -0.72, -1.97], [10.1, -7.7, -1.97], [7.28, -12.76, -2.609], [2.0, -10.8, 2.74], [0.5, -6.5, 1.169]]
        
        partition_costs = []
        for i in range(len(trace)-1):
            partition_costs.append( dist(trace[i], trace[i+1]) )
        partition_costs.append(0)
       
        #prepare trace_pub_array to publish the trace
        trace_pub_array = []
        for i in range(len(trace)):
            trace_pub_array.append( trace[i][0] )
            trace_pub_array.append( trace[i][1] )
            trace_pub_array.append( trace[i][2] )
        trace_for_publishing = Float32MultiArray(data=trace_pub_array)
        
        ################################################
        #start analysis
        
        while not rospy.is_shutdown() and ( not stop_theta(self.odom_xyp, trace[len(trace)-2], trace[len(trace)-1]) or i_goal < (len(trace)-1) ):
            #init publishes
            dist_2_goal = Float32()
            partition_dist_2_goal = Float32()
            
            #next goal?
            if stop_theta(self.odom_xyp, trace[i_goal-1], trace[i_goal]) and i_goal < (len(trace)-1):
                i_goal += 1
            
            ##missed partition goal -> next partition goal
            #if i_goal < (len(trace)-1):
            #    if dist(self.odom_xyp, trace[i_goal]) > dist(self.odom_xyp, trace[i_goal+1]) and eucl_dist(self.odom_xyp, trace[i_goal]) > eucl_dist(self.odom_xyp, trace[i_goal+1]):
            #        i_goal += 1 #use next partition goal on trace
            
            #calc dists
            #dist 2 next partition goals (pg)
            dist_2_pg1 = dist(self.odom_xyp, trace[i_goal])
            if i_goal < (len(trace)-1):
                dist_2_pg2 = dist(self.odom_xyp, trace[i_goal+1])
            else:
                dist_2_pg2 = dist(self.odom_xyp, trace[i_goal])
            
            #theta
            if not start_theta(self.odom_xyp, trace[i_goal-1], trace[i_goal]):
                theta = 0
            elif stop_theta(self.odom_xyp, trace[i_goal-1], trace[i_goal]):
                theta = 1
            else:
                theta = calc_theta(self.odom_xyp, trace[i_goal-1], trace[i_goal])
                if theta > 1:
                    theta = 1
                if theta < 0:
                    theta = 0
            self.theta.data = theta
            
            #calc total dist
            d = theta*dist_2_pg2 + (1-theta)*(dist_2_pg1+partition_costs[i_goal]) #dist 2 (second) next goal
            p_d = d #partition dist 2 goal
            for i in range(i_goal+1, len(trace), 1):
                d += partition_costs[i] #total dist 2 goal
            
            #publish
            partition_dist_2_goal = p_d
            dist_2_goal.data = d
            pub_partition_dist.publish(partition_dist_2_goal)
            pub_dist.publish(dist_2_goal)
            pub_theta.publish(self.theta)
            self.partition_goal_xyp.data = trace[i_goal]
            pub_partition_goal_xyp.publish(self.partition_goal_xyp)
            pub_trace.publish(trace_for_publishing)
            

    def odom_callback(self, data):
        self.odom_xyp[0] = data.pose.pose.position.x
        self.odom_xyp[1] = data.pose.pose.position.y
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        euler = euler_from_quaternion(quat)
        self.odom_xyp[2] = euler[2]



####################################################
#           start node
####################################################  

if  __name__=="__main__":
    rospy.init_node("target_calculation")
    try:
        node=TargetCalculation()

    except rospy.ROSInterruptException:
        pass