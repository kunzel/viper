#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.collections import PolyCollection
from matplotlib.colors import colorConverter
import matplotlib.pyplot as plt
import numpy as np

import random
import jsonpickle
import rospy
import matplotlib

def cc(arg):
    return colorConverter.to_rgba(arg, alpha=0.6)


INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

runs = [0]
rhos = ['1.0']
MAX_REWARD = 2637
N = [50] #500
TIME = [30, 60, 90, 120, 180, 240]
BEST_M = [5,10,15, 20, 30, 40, 50, 165]

    # for N in 50 100 500 1000  
    # do
    #     for TIME in 60 120 180 240 
    #     do
    #         for BEST_M in 20 30 40 50 

max_p_time = dict()
run_stats = dict()
for run in runs:
    for rho in rhos:
        for n in N:
            for m in BEST_M:
                for t in TIME:
                    max_p_time[(str(run), str(rho), str(n), str(t))] = 0
                    INPUT_FILE = INPUT_FILE_DIR + '/ViewDEP-AlgIJCAIBESTM-N' + str(n) + '-T' + str(t) + '-M' + str(m) + '-Rho' + str(rho) + '-Run' + str(run)+ '-BEST.json'
                    with open(INPUT_FILE, "r") as input_file:
                        json_data = input_file.read()
                        plan = jsonpickle.decode(json_data)
                        p_time = plan.planning_time
                        if p_time > max_p_time[(str(run), str(rho), str(n), str(t))]:
                            max_p_time[(str(run), str(rho), str(n), str(t))] = p_time
                        print "N:", n, "M:", m, "T:", t, "PLANNING TIME:", plan.planning_time
                        run_stats[(str(run), str(rho), str(n), str(t), str(m))] = plan


for n in N:
    
    print n
    fig = plt.figure()
    ax = fig.gca(projection='3d')


    verts = []
    zs = TIME
    for z in zs:
        import copy
        xs = copy.deepcopy(BEST_M)
        ys = []
        ys.append(0)
        for x in xs:
            plan = run_stats[(str(0), str(1.0), str(n), str(z), str(x))]
            r = float(plan.planning_time)/ max_p_time[(str(0), str(1.0), str(n), str(z))] # float(plan.reward)/MAX_REWARD 
            ys.append(r)

        ys.append(0)
        #ys.append(ys[-1])
        ys = np.array(ys)
        #np.random.rand(len(xs))
        #ys[0], ys[-1] = 0, 0

        # align xs
        xs.insert(0,0)
        xs.append(BEST_M[-1]+BEST_M[0])
        xs=np.array(xs)
        verts.append(list(zip(xs, ys)))

    poly = PolyCollection(verts, facecolors=[cc('r'), cc('g'), cc('b'),
                                             cc('y'), cc('r'), cc('g')])
    poly.set_alpha(0.7)
    ax.add_collection3d(poly, zs=zs, zdir='y')

    ax.set_xlabel('BEST_M')
    ax.set_xlim3d(BEST_M[0]-BEST_M[0], BEST_M[-1]+BEST_M[0])
    ax.set_ylabel('TIME')
    ax.set_ylim3d(TIME[0]-TIME[0], TIME[-1]+TIME[0])
    ax.set_zlabel('PLANNING TIME')
    ax.set_zlim3d(0, 1)
    
    plt.show()
