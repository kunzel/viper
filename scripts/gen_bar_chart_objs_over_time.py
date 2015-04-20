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

rospy.init_node('gen_bar_chart')

INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

#scns = ['A','B']
scns = ['B']
rhos = ['0.0', '0.5', '1.0', '2.0']
runs = range(10)

run_stats = dict()
for scn in scns:
    for rho in rhos:
        for run in runs:
            INPUT_FILE = INPUT_FILE_DIR + '/plans-' + scn + '-' + rho  + '-' + str(run) + '-EXEC_LOG.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(scn,rho,str(run))] = jsonpickle.decode(json_data)
rospy.loginfo("Loaded %s stats")

font = {'family' : 'normal',
#        'weight' : 'bold',
        'size'   : 32}

matplotlib.rc('font', **font)

times = range(10, 91, 10)
#times = range(0, 221, 20) # [30, 60, 90, 120, 150, 180, 210, 240] 
n_groups = len(times) #len(scns)

means = dict()
stds  = dict()
for rho in rhos:
    means[rho] = []
    stds[rho]  =  []
    for scn in scns:
        for t in times:
            objs = {}  
            nums = []
            for run in runs:
                t_objs = []
                if run not in objs:
                    objs[run] = []
                stats = run_stats[(scn,rho,str(run))]
                for o in stats['found_objs']:
                    if o[1]['name'] not in objs[run] and o[0].to_sec() <= t:
                        t_objs.append(o[1]['name'])
                        objs[run].append(o[1]['name'])
                nums.append(len(t_objs))
            # if len(means[rho]) == 0:
            #     last_mean = 0.0
            # else:
            #     last_mean = means[rho][len(means[rho])-1]
            # if len(stds[rho]) == 0:
            #     last_std = 0.0
            # else:
            #     last_std = stds[rho][len(stds[rho])-1]
            means[rho].append((np.mean(nums)))
            stds[rho].append((np.std(nums)))

means_rho00 = tuple(means['0.0'])
std_rho00 = tuple(stds['0.0'])
        
means_rho05 = tuple(means['0.5'])
std_rho05 = tuple(stds['0.5'])

means_rho10 = tuple(means['1.0'])
std_rho10 = tuple(stds['1.0'])

means_rho20 = tuple(means['2.0'])
std_rho20 = tuple(stds['2.0'])


fig, ax = plt.subplots()

index = np.arange(n_groups)
bar_width = 0.2

opacity = 0.4 
error_config = {'ecolor': '0.3'}

colors = ['blue', 'red', 'yellow', 'green']

cidx = 0
rects0 = plt.bar(index, means_rho00, bar_width,
                  alpha=opacity,
                  color=colors[cidx],
                  yerr=std_rho00,
                  error_kw=error_config,
                  label=r'$\varrho$ = 0.0')


cidx += 1
rects1 = plt.bar(index + bar_width, means_rho05, bar_width,
                  alpha=opacity,
                  color=colors[cidx],
                  yerr=std_rho05,
                  error_kw=error_config,
                  label=r'$\varrho$ = 0.5')


cidx += 1
rects2 = plt.bar(index + 2 * bar_width, means_rho10, bar_width,
                 alpha=opacity,
                 color=colors[cidx],
                 yerr=std_rho10,
                 error_kw=error_config,
                 label=r'$\varrho$ = 1.0')

cidx += 1
rects3 = plt.bar(index + 3* bar_width, means_rho20, bar_width,
                 alpha=opacity,
                 color=colors[cidx],
                 yerr=std_rho20,
                 error_kw=error_config,
                 label=r'$\varrho$ = 2.0')

ax.set_ylim((0,4))
plt.xlabel('Time (in seconds)')
plt.ylabel('Average number of found objects')
#plt.title('Average number of found objects by parametrisation (rho) over time')
plt.xticks(index + 2 * bar_width, tuple(times))
plt.legend(loc=2)
plt.tight_layout()
plt.show()
