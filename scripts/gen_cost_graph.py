#!/usr/bin/env python
"""
Demo of the errorbar function, including upper and lower limits
"""
import numpy as np
import matplotlib.pyplot as plt
import random
import jsonpickle
import rospy
import matplotlib

font = {'family' : 'normal',
#        'weight' : 'bold',
        'size'   : 32}

matplotlib.rc('font', **font)


rospy.init_node('gen_cost_graph')

INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

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

n_groups = len(scns)


max_mean = 0.0
means = dict()
stds  = dict()
keys = []
for rho in rhos:
    means[rho] = [0.0]
    stds[rho]  = [0.0]
    times = {}
    for scn in scns:

        for run in runs:
            print "Run: ", run
            stats = run_stats[(scn,rho,str(run))]
            start_time = stats['start_time']
            len(stats['run_times'])
            i = 0
            for v in stats['run_times']:
                if i not in times:
                    times[i] = []
                times[i].append(v[0].to_sec())
                i += 1
    keys = times.keys()
    for k in sorted(keys):
        mean = np.mean(times[k])
        if mean > max_mean:
            max_mean = mean
        means[rho].append(mean)
        stds[rho].append(np.std(times[k]))


max_view = max(keys)
# example data


x =  np.arange(0.0, max_view+2, 1.0)

y_rho_00 = np.array(means['0.0'])
y_rho_05 = np.array(means['0.5'])
y_rho_10 = np.array(means['1.0'])
y_rho_20 = np.array(means['2.0'])

yerr_rho_00 = tuple(stds['0.0'])
yerr_rho_05 = tuple(stds['0.5'])
yerr_rho_10 = tuple(stds['1.0'])
yerr_rho_20 = tuple(stds['2.0'])

ls = 'dotted'

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


colors = ['blue', 'red', 'yellow', 'green']
cidx = 0
# standard error bars
plt.errorbar(x, y_rho_00, yerr=yerr_rho_00, marker='^', markersize=20, ls=ls, color=colors[cidx], label=r'$\varrho$ = 0.0')

cidx += 1
plt.errorbar(x, y_rho_05,yerr=yerr_rho_05, marker='>', markersize=20, ls=ls, color=colors[cidx], label=r'$\varrho$ = 0.5')

cidx += 1
plt.errorbar(x, y_rho_10,yerr=yerr_rho_10, marker='<', markersize=20,  ls=ls, color=colors[cidx], label=r'$\varrho$ = 1.0')

cidx += 1
plt.errorbar(x, y_rho_20,yerr=yerr_rho_20, marker='v', markersize=20,  ls=ls, color=colors[cidx], label=r'$\varrho$ = 2.0')

plt.plot(np.array([90 for i in range(26)]), '--k')
# # including upper limits
# uplims = np.zeros(x.shape)
# uplims[[1, 5, 9]] = True
# # including lower limits
# lolims = np.zeros(x.shape)
# lolims[[2, 4, 8]] = True

# # including upper and lower limits
# plt.errorbar(x, y+1.5, marker='o', ms=8, yerr=yerr,
#              lolims=lolims, uplims=uplims, ls=ls, color='magenta')

ax.set_ylim((0, max_mean + 10))
ax.set_xlim((0, max_view + 1))
plt.xlabel('Views')
plt.ylabel('Average cost (time in seconds)')
#plt.title('Average cost by view and parameterisation (rho)')
plt.legend(loc=2)
plt.show()

