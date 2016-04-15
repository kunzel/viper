#!/usr/bin/env python
"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
import jsonpickle
#import rospy
import random

import matplotlib

font = {#'family' : 'normal',
#        'weight' : 'bold',
         'size'   : 34}

matplotlib.rc('font', **font)

MAX_OBJS= [4,8,16,24]

#rospy.init_node('gen_bar_chart')

INPUT_FILE_DIR = './' #rospy.get_param('~input_file_dir', './')

ALG = ['FIXED1','FIXED2','TS']
ENV = ['env1', 'env2', 'env3', 'env4',]
T = [60, 120, 180, 240, 300, 360]


# ts_policy = [# env1
# {
#      60 : (10, 70, 60),
#  120 : (30, 5, 90),
#  180 : (100, 15, 150),
#  240 : (5, 15, 180),
#  300 : (20, 80, 270),
#  360 : (10, 30, 270)
    
#  # 60 : (30, 15, 60),
#  # 120 : (30, 20, 120),
#  # 180 : (20, 60, 180),
#  # 240 : (5, 30, 210),
#  # 300 : (100, 20, 210),
#  # 360 : (5, 15, 240)
# }
# ]


# TS policy
ts_policy = [# env1
{
 60 : (10, 70, 60),
 120 : (30, 5, 90),
 180 : (100, 15, 150),
 240 : (5, 15, 180),
 300 : (20, 80, 270),
 360 : (10, 30, 270)
}
,
    {
 60 : (10, 70, 60),
 120 : (30, 5, 90),
 180 : (100, 15, 150),
 240 : (5, 15, 180),
 300 : (20, 80, 270),
 360 : (10, 30, 270)
},
#     { # env 2
#  60 : (10, 70, 60),
#  120 : (30, 5, 90),
#  180 : (100, 30, 120),
#  240 : (5, 30, 210),
#  300 : (100, 20, 210),
#  360 : (5, 15, 240)
# },
{ #env 3
 60 : (30, 15, 60),
 120 : (30, 20, 120),
 180 : (20, 60, 180),
 240 : (5, 30, 210),
 300 : (100, 20, 210),
 360 : (5, 15, 240)
},
{ #env 4
 60 : (30, 15, 60),
 120 : (30, 20, 120),
 180 : (20, 60, 180),
 240 : (5, 30, 210),
 300 : (100, 20, 210),
 360 : (5, 15, 240)
}
# { # env4 
#  60 : (750, 20, 60),
#  120 : (500, 80, 60),
#  180 : (1000, 30, 60),
#  240 : (1000, 60, 60),
#  300 : (100, 20, 210),
#  360 : (5, 15, 240)
# }
]         

run_stats = dict()
for alg in ALG:
    for t in T:
        if alg == 'FIXED1':
            for env in ENV:
                n = 20
                m = 30
                INPUT_FILE = INPUT_FILE_DIR + env +'/plans/ViewDEP-AlgVP_S-N' + str(n) + '-T' + str(t) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
                with open(INPUT_FILE, "r") as input_file:
                    json_data = input_file.read()
                    run_stats[(alg,env,t)] = jsonpickle.decode(json_data)
                
        elif alg == 'FIXED2':
            for env in ENV:
                n = 10
                m = 30
                INPUT_FILE = INPUT_FILE_DIR + env +'/plans/ViewDEP-AlgVP_S-N' + str(n) + '-T' + str(t) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  

                with open(INPUT_FILE, "r") as input_file:
                    json_data = input_file.read()
                    run_stats[(alg,env,t)] = jsonpickle.decode(json_data)
        elif alg == 'TS':
            for env in ENV:
                n, m, tx = ts_policy[ENV.index(env)][t]  
                INPUT_FILE = INPUT_FILE_DIR + env + '/plans/ViewDEP-AlgVP_S-N' + str(n) + '-T' + str(tx) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
                with open(INPUT_FILE, "r") as input_file:
                    json_data = input_file.read()
                    run_stats[(alg,env,t)] = jsonpickle.decode(json_data)

print("loaded stats")
#rospy.loginfo("Loaded %s stats")




n_groups = len(T)

means = dict()
stds  = dict()
for alg in ALG:
    means[alg] = []
    stds[alg]  =  []
    for t in T:
        nums = []
        for env in ENV:
            objs = {}  
            for run in [0]:
                t_objs = []
                if run not in objs:
                    objs[run] = []
                stats = run_stats[(alg,env,t)]
                for o in stats['found_objs']:
                    if o[1]['type'] == 'cup' and o[1]['name'] not in objs[run] and o[0].to_sec() <= t  - stats['planning_time']:
                        t_objs.append(o[1]['name'])
                        objs[run].append(o[1]['name'])

                objs = len(t_objs)
                nums.append(((float(len(t_objs)))/MAX_OBJS[ENV.index(env)]*100.00))
        print nums
        means[alg].append((np.mean(nums))+1)
        stds[alg].append((np.std(nums)))

        
means_alg00 = tuple(means['FIXED1'])
std_alg00 = tuple(stds['FIXED1'])

means_alg05 = tuple(means['FIXED2'])
std_alg05 = tuple(stds['FIXED2'])

means_alg10 = tuple(means['TS'])
std_alg10 = tuple(stds['TS'])


fig, ax = plt.subplots()

index = np.arange(n_groups)
bar_width = 0.2

opacity = 0.4 
error_config = {'ecolor': '0.3'}

colors = ['yellow', 'red', 'blue', 'green']

cidx = 0
rects0 = plt.bar(index, means_alg00, bar_width,
                  #alpha=opacity,
                  color=colors[cidx],
                  yerr=std_alg00,
                  error_kw=error_config,
                  label=r'fixed ($n_s=20, m=30, \widehat{T}=T$)')


cidx += 1
rects1 = plt.bar(index + bar_width, means_alg05, bar_width,
                  #alpha=opacity,
                  color=colors[cidx],
                  yerr=std_alg05,
                  error_kw=error_config,
                  label=r'fixed ($n_s=10, m=30, \widehat{T}=T$)')


cidx += 1
rects2 = plt.bar(index + 2 * bar_width, means_alg10, bar_width,
                 #alpha=opacity,
                 color=colors[cidx],
                 yerr=std_alg10,
                 error_kw=error_config,
                 label=r'adaptive')

# cidx += 1
# rects3 = plt.bar(index + 3* bar_width, means_alg20, bar_width,
#                  alpha=opacity,
#                  color=colors[cidx],
#                  yerr=std_alg20,
#                  error_kw=error_config,
#                  label=r'time-sensitive')

ax.set_ylim((0,100))
plt.xlabel('$T$ (in seconds)')
plt.ylabel('Average % of successful runs')
#plt.title('Average number of found objects by parametrisation (alg) over time')
plt.xticks(index + 1.5 *bar_width, tuple(T))
plt.legend(loc=2)
plt.tight_layout()
plt.show()

