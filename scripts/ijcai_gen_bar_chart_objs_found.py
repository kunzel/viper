#!/usr/bin/env python
"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
import jsonpickle
import rospy
import random

import matplotlib

font = {#'family' : 'normal',
#        'weight' : 'bold',
         'size'   : 32}

matplotlib.rc('font', **font)

MAX_OBJS = 10

rospy.init_node('gen_bar_chart')

INPUT_FILE_DIR = rospy.get_param('~input_file_dir', './')

ALG = ['ALOOFS','ALOOFBESTM','TS']
T = [30, 60, 120, 180, 240, 300]

# TS policy
ts_policy = {30: (10, 30, 30),  
             60: (20, 50, 45),
             120: (30, 100, 120),
             180: (30, 100, 150),
             240: (20, 100, 210),
             300: (20, 100, 210)}

run_stats = dict()
for alg in ALG:
    for t in T:
        if alg == 'ALOOFS':
            n = 250
            m = -1
            INPUT_FILE = INPUT_FILE_DIR + 'ViewDEP-AlgALOOFBESTM-N' + str(n) + '-T' + str(t) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(alg,t)] = jsonpickle.decode(json_data)
        elif alg == 'ALOOFFAST':
            n = 10
            m = 30
            INPUT_FILE = INPUT_FILE_DIR + 'ViewDEP-AlgALOOFBESTM-N' + str(n) + '-T' + str(t) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(alg,t)] = jsonpickle.decode(json_data)
        elif alg == 'ALOOFBESTM':
            n = 100
            m = 50
            INPUT_FILE = INPUT_FILE_DIR + 'ViewDEP-Alg'+alg+'-N' + str(n) + '-T' + str(t) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(alg,t)] = jsonpickle.decode(json_data)
        elif alg == 'TS':
            n, m, tx = ts_policy[t]  
            INPUT_FILE = INPUT_FILE_DIR + 'ViewDEP-Alg'+alg+'-N' + str(n) + '-T' + str(tx) + '-M'+ str(m) + '-Rho1.0-Run0-BEST-EXEC_LOG.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(alg,t)] = jsonpickle.decode(json_data)
            
rospy.loginfo("Loaded %s stats")

n_groups = len(T)

means = dict()
stds  = dict()
for alg in ALG:
    means[alg] = []
    stds[alg]  =  []
    for t in T:
        objs = {}  
        nums = []
        for run in [0]: #runs:
            t_objs = []
            if run not in objs:
                objs[run] = []
            stats = run_stats[(alg,t)]
            for o in stats['found_objs']:
                if o[1]['type'] == 'cup' and o[1]['name'] not in objs[run] and o[0].to_sec() <= t  - stats['planning_time']:
                    t_objs.append(o[1]['name'])
                    objs[run].append(o[1]['name'])

            objs = len(t_objs)
            print objs
            nums.append(((float(len(t_objs)))/MAX_OBJS*100.00) + 1)
            # if len(means[alg]) == 0:
            #     last_mean = 0.0
            # else:
            #     last_mean = means[alg][len(means[alg])-1]
            # if len(stds[alg]) == 0:
            #     last_std = 0.0
            # else:
            #     last_std = stds[alg][len(stds[alg])-1]
        means[alg].append((np.mean(nums)))
        stds[alg].append((np.std(nums)))

# means_alg00 = tuple(means['ALOOFFAST'])
# std_alg00   = tuple(stds['ALOOFFAST'])
        
means_alg00 = tuple(means['ALOOFS'])
std_alg00 = tuple(stds['ALOOFS'])

means_alg05 = tuple(means['ALOOFBESTM'])
std_alg05 = tuple(stds['ALOOFBESTM'])

means_alg10 = tuple(means['TS'])
std_alg10 = tuple(stds['TS'])


fig, ax = plt.subplots()

index = np.arange(n_groups)
bar_width = 0.2

opacity = 0.4 
error_config = {'ecolor': '0.3'}

colors = ['blue', 'red', 'yellow', 'green']

cidx = 0
rects0 = plt.bar(index, means_alg00, bar_width,
                  #alpha=opacity,
                  color=colors[cidx],
                  yerr=std_alg00,
                  error_kw=error_config,
                  label=r'fixed ($n_s=250, m=|S|, \widehat{T}=T$)')


cidx += 1
rects1 = plt.bar(index + bar_width, means_alg05, bar_width,
                  #alpha=opacity,
                  color=colors[cidx],
                  yerr=std_alg05,
                  error_kw=error_config,
                  label=r'fixed ($n_s=100, m=50, \widehat{T}=T$)')


cidx += 1
rects2 = plt.bar(index + 2 * bar_width, means_alg10, bar_width,
                 #alpha=opacity,
                 color=colors[cidx],
                 yerr=std_alg10,
                 error_kw=error_config,
                 label=r'time-sensitive')

# cidx += 1
# rects3 = plt.bar(index + 3* bar_width, means_alg20, bar_width,
#                  alpha=opacity,
#                  color=colors[cidx],
#                  yerr=std_alg20,
#                  error_kw=error_config,
#                  label=r'time-sensitive')

ax.set_ylim((0,100))
plt.xlabel('$T$ (in seconds)')
plt.ylabel('% of successful runs')
#plt.title('Average number of found objects by parametrisation (alg) over time')
plt.xticks(index + 1.5 *bar_width, tuple(T))
plt.legend(loc=2)
plt.tight_layout()
plt.show()

