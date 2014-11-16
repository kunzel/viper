#!/usr/bin/env python
"""
Bar chart demo with pairs of bars grouped for easy comparison.
"""
import numpy as np
import matplotlib.pyplot as plt
import jsonpickle
import rospy
import random

rospy.init_node('gen_bar_chart')

INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

#scns = ['A','B']
scns = ['test','test']
rhos = ['rho_00', 'rho_05', 'rho_10', 'rho_20']
runs = range(1)

run_stats = dict()
for scn in scns:
    for rho in rhos:
        for run in runs:
            INPUT_FILE = INPUT_FILE_DIR + '/' + scn + '-' + rho  + '-' + str(run) + '.json'  
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(scn,rho,str(run))] = jsonpickle.decode(json_data)
rospy.loginfo("Loaded %s stats")

n_groups = len(scns)

means = dict()
stds  = dict()
for rho in rhos:
    means[rho] = []
    stds[rho]  =  []
    for scn in scns:
        nums = []
        for run in runs:
            stats = run_stats[(scn,rho,str(run))]
            objs = []  
            for o in stats['found_objs']:
                if o[1]['name'] not in objs:
                    objs.append(o[1]['name'])
            nums.append(len(objs))
        means[rho].append(np.mean(nums))
        stds[rho].append(np.std(nums)+random.random())

means_rho00 = tuple(means['rho_00'])
std_rho00 = tuple(stds['rho_00'])
        
means_rho05 = tuple(means['rho_05'])
std_rho05 = tuple(stds['rho_05'])

means_rho10 = tuple(means['rho_10'])
std_rho10 = tuple(stds['rho_10'])

means_rho20 = tuple(means['rho_20'])
std_rho20 = tuple(stds['rho_20'])


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
                  label='rho = 0.0')


cidx += 1
rects1 = plt.bar(index + bar_width, means_rho05, bar_width,
                  alpha=opacity,
                  color=colors[cidx],
                  yerr=std_rho05,
                  error_kw=error_config,
                  label='rho = 0.5')


cidx += 1
rects2 = plt.bar(index + 2 * bar_width, means_rho10, bar_width,
                 alpha=opacity,
                 color=colors[cidx],
                 yerr=std_rho10,
                 error_kw=error_config,
                 label='rho = 1.0')

cidx += 1
rects3 = plt.bar(index + 3* bar_width, means_rho20, bar_width,
                 alpha=opacity,
                 color=colors[cidx],
                 yerr=std_rho20,
                 error_kw=error_config,
                 label='rho = 2.0')

plt.xlabel('Scenario')
plt.ylabel('Average number of found objects')
plt.title('Average number of found objects by scene and parametrisation (rho)')
plt.xticks(index + 2 * bar_width, tuple(scns))
plt.legend()
plt.tight_layout()
plt.show()
