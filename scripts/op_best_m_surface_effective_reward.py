#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState 

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import PolyCollection
from matplotlib.colors import colorConverter
import matplotlib.pyplot as plt
import numpy as np

import random
import jsonpickle
import rospy
import matplotlib

import viper.robots.scitos_coverage
robot = viper.robots.scitos_coverage.ScitosRobot()


font = {#'family' : 'normal',
#        'weight' : 'bold',
        'size'   : 32}

matplotlib.rc('font', **font)

rospy.init_node('figure')
    
INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

# OLD G4S!!!!
#MAX_REWARD = 11128

surface = dict()

#alg = "IRLABBESTM"
#MAX_REWARD = 2637

# NEW G4S
alg = "G4SBESTM"
MAX_REWARD = 10498

#alg = "ALOOFBESTM"
#MAX_REWARD = 4156


runs = [0]
rhos = ['1.0']



# variables that effect planning time
N = [5, 10, 20, 30, 40, 50, 100, 250, 500] #, 1000, 2000] 
TIME =  [30, 60, 120, 180, 240, 300] #, 600]
time_colors = [('r', '^'), ('g', '>'), ('b', 'v'), ('y', '<'),('m', '.'),('c', 'o'),('w','+')] #('k','*')]
BEST_M = [5, 10, 15, 20, 30, 50, 100, 125, -1] #165, 467] #[5, 10, 15, 20, 30, 50, 100, 125, 165]

#TWINDOW = [30, 60, 120, 180, 240, 300] # 600
TWINDOW = [120] #[30,60,120,180,240,300] # 600


INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', '../view_costs.json')
INPUT_FILE = rospy.get_param('~input_file', '../view_keys.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))


view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs")

try:
    rospy.loginfo("Wait for /robot_pose")
    robotpose_msg = rospy.wait_for_message("/robot_pose", Pose, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /robot_pose")

try:
    rospy.loginfo("Wait for /ptu/state")
    jointstate_msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /ptu/state")

    
# Add current pose to view_costs (key: '-1')
current_pose = robotpose_msg
rospy.loginfo("Current pose: %s" % current_pose)
current_ptu_state = JointState()
current_ptu_state.name = ['pan', 'tilt']
current_ptu_state.position = [jointstate_msg.position[jointstate_msg.name.index('pan')],jointstate_msg.position[jointstate_msg.name.index('tilt')]]
current_view =  viper.robots.scitos_coverage.ScitosView("-1", current_pose, current_ptu_state, None) # ptu pose is not needed for cost calculation
vcosts = dict()
for v in views:
    cost = robot.cost(current_view,v)
    vcosts[v.ID] = cost
    view_costs[v.ID][current_view.ID] = cost  

view_costs[current_view.ID] = vcosts
view_costs[current_view.ID][current_view.ID] = 0  


    
max_p_time = dict()
run_stats = dict()
for run in runs:
    for rho in rhos:
        for n in N:
            for m in BEST_M:
                for t in TIME:
                    max_p_time[(str(run), str(rho), str(n), str(t))] = 0
                    INPUT_FILE = INPUT_FILE_DIR + '/ViewDEP-Alg' + alg +'-N' + str(n) + '-T' + str(t) + '-M' + str(m) + '-Rho' + str(rho) + '-Run' + str(run)+ '-BEST.json'
                    with open(INPUT_FILE, "r") as input_file:
                        json_data = input_file.read()
                        plan = jsonpickle.decode(json_data)
                        p_time = plan.planning_time
                        if p_time > max_p_time[(str(run), str(rho), str(n), str(t))]:
                            max_p_time[(str(run), str(rho), str(n), str(t))] = p_time
                        print "N:", n, "M:", m, "T:", t, "PLANNING TIME:", plan.planning_time
                        run_stats[(str(run), str(rho), str(n), str(t), str(m))] = plan


fig = plt.figure()


plots = [111] #[231,232,233,234,235,236] #[131, 132, 133] #[231,232,233,234,235,236]
p = 0
for twin in TWINDOW:
    print p, twin
    
    ax2 = fig.add_subplot(plots[p], projection='3d')
    p +=1
    max_r = dict()
    max_tx = dict()
    max_t = 0
    min_t = dict()
    for t in TIME:

        if t > twin:
            continue
        
        xs = []
        ys = []
        zs = []
        c, ma = time_colors[TIME.index(t)]
        
        for n in N:
            for m in BEST_M:
                plan = run_stats[(str(0), str(1.0), str(n), str(t), str(m))]
                pt = plan.planning_time
                if pt > twin: # and False:
                    r = 0
                else:
                    r = 0
                    r_old = 0
                    cost_old = 0
                    et = 0
                    i = -1
                    keys = dict()
                    while  i < len(plan.views) and (pt + et) <= twin:
                        r_old =  r
                        cost_old =  pt + et # et
                        v = plan.views[i]
                        for k in v.get_keys():
                            if k not in keys:
                                keys[k] = True
                                r += 1
                        et +=  view_costs[plan.views[i-1].ID][plan.views[i].ID] # cost to move from i to i+1
                        i += 1
                    
                    r = float(r_old)/MAX_REWARD 
                    
                xs.append(n)
                ys.append(m)
                zs.append(r)

                

                cost = cost_old
                
                # if (n,m) not in min_t:
                #     min_t[(n,m)] = cost
                # else:
                #     if cost < min_t:
                #         min_t[(n,m)] = cost 
                
                if (n,m) not in max_r:
                    max_r[(n,m)] = r
                    max_tx[(n,m)] = t
                    min_t[(n,m)] = cost
                    if cost > max_t:
                        max_t = cost

                else:
                    if r > max_r[(n,m)]:
                        max_r[(n,m)] = r
                        max_tx[(n,m)] = t
                        min_t[(n,m)] = cost
                        if cost > max_t:
                            max_t = cost
        #ax.scatter(xs, ys, zs, c=c, marker=ma)

    X = []
    Y = []
    Z = []

    C = []

    RC = []
        
    for n in N:
        xtmp = []
        ytmp = []
        ztmp = []
        ctmp = []
        rctmp = []
        for m in BEST_M:
            xtmp.append(n)
            if m == -1:
                if alg == 'IRLABBESTM':
                    ytmp.append(165) # IRLAB
                elif alg == 'G4SBESTM':
                    ytmp.append(314) # G4S NEW
                elif alg == 'ALOOFBESTM':
                    ytmp.append(185) # ALOOF
                else:
                    print 'ERROR WITH ENV SETTING'
            else:
                ytmp.append(m)
                
            ztmp.append(max_r[(n,m)])
            ctmp.append((float(min_t[(n,m)])/max_t))
            rctmp.append(max_r[(n,m)]/((float(min_t[(n,m)])/max_t))+0.01)

            surface[(twin,n,m,max_tx[(n,m)],'R')] = max_r[(n,m)]
            surface[(twin,n,m,max_tx[(n,m)],'C')] = (float(min_t[(n,m)])/max_t)
            surface[(twin,n,m,max_tx[(n,m)],'R/C')] = max_r[(n,m)]/((float(min_t[(n,m)])/max_t))+0.01
        
        X.append(xtmp)
        Y.append(ytmp)
        Z.append(ztmp)
        C.append(ctmp)
        RC.append(rctmp)
    #print X, Y, Z
    
    from mpl_toolkits.mplot3d import axes3d
    import matplotlib.pyplot as plt
    from matplotlib import cm

    #fig2 = plt.figure()
    #ax2 = fig2.gca(projection='3d')
    #X, Y, Z = axes3d.get_test_data(0.05)
    nZ = []

    ax2.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2, linewidth=1, edgecolors='b', color='b')
    #ax2.plot_surface(X, Y, C, rstride=1, cstride=1, alpha=0.2, linewidth=1, edgecolors='r', color='r')
    #ax2.plot_surface(X, Y, RC, rstride=1, cstride=1, alpha=0.2, linewidth=1, edgecolors='g', color='g')

    #cset = ax2.contour(X, Y, Z, zdir='z', offset=0, cmap=cm.coolwarm)
    #cset = ax2.contour(X, Y, Z, zdir='x', offset=5, cmap=cm.coolwarm)
    #cset = ax2.contour(X, Y, Z, zdir='y', offset=0, cmap=cm.coolwarm)

    fs = 42
    stepsize = 100
    
    ax2.set_title('$T$='+str(twin), fontsize=fs)
    ax2.set_xlabel('$n_s$', fontsize=fs)
    start, end = ax2.get_xlim()
    ax2.xaxis.set_ticks(np.arange(start, end, stepsize))
    #ax.set_xlim(0)
    ax2.set_ylabel('$m$', fontsize=fs)
    start, end = ax2.get_ylim()
    ax2.yaxis.set_ticks(np.arange(start, end, stepsize))
    #ax.set_ylim(-40, 40)
    ax2.set_zlabel('$R_T$', fontsize=fs)
    #ax2.set_zlim(0, 1)

plt.show()


with open("./Surface-" + alg + ".json", "w") as outfile:
    json_data = jsonpickle.encode(surface)
    outfile.write(json_data)
       

    # ax.set_xlabel('#paths (N)')
    # ax.set_ylabel('best views (M)')
    # ax.set_zlabel('Reward (P_t + E_t <= T)')
    #plt.show()

    #############################################

    
        
    # poly = PolyCollection(verts, facecolors=[cc('r'), cc('g'), cc('b'),
    #                                          cc('y'), cc('r'), cc('g')])
    # poly.set_alpha(0.7)
    # ax.add_collection3d(poly, zs=zs, zdir='y')

    # ax.set_xlabel('BEST_M')
    # ax.set_xlim3d(BEST_M[0]-BEST_M[0], BEST_M[-1]+BEST_M[0])
    # ax.set_ylabel('#Views (N)')
    # ax.set_ylim3d(N[0]-N[0], N[-1]+N[0])
    # ax.set_zlabel('PLANNING TIME')
    # ax.set_zlim3d(0, 1)
    
    # plt.show()



