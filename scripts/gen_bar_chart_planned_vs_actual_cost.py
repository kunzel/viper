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

scns = ['A']
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

n_groups = 2 #len(scns)

means = dict()
stds  = dict()
for rho in rhos:
    means[rho] = []
    stds[rho]  =  []
    for scn in scns:
        actual = []
        planned = []
        for run in runs:
            stats = run_stats[(scn,rho,str(run))]
            actual.append(stats['end_time'].to_sec())
            planned.append(stats['expected_costs'])
        means[rho].append(np.mean(planned))
        means[rho].append(np.mean(actual))
        stds[rho].append(np.std(planned))
        stds[rho].append(np.std(actual))

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

plt.xlabel('Cost type')
plt.ylabel('Average cost (in seconds))')
plt.title('Average cost by parametrisation (rho) over ten runs')
plt.xticks(index + 2 * bar_width, ('Planned','Actual'))
plt.legend()
plt.tight_layout()
plt.show()
