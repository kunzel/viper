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
#        'size'   : 32}
        'size'   : 24}        

matplotlib.rc('font', **font)


rospy.init_node('gen_rewards_vs_baseline')

INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')
TIME_WINDOW = rospy.get_param('~time_window', '20')


rhos = ['4.0', '4.0', '4.0', '4.0', '4.0'] 
Ns =  ['5', '10', '20', '50', '100'] #, '5000' ]#,'16000','32000', '64000']
runs = [19] # range(1) 

run_stats = dict()
for rho in rhos:
    for n in Ns:
        for run in runs:
            INPUT_FILE = INPUT_FILE_DIR + '/ViewDEP-AlgSRCBESTM-N' + n + '-T' + TIME_WINDOW+ '-Rho' + rho+ '-Run' + str(run)+ '-BEST.json'
            with open(INPUT_FILE, "r") as input_file:
                json_data = input_file.read()
                run_stats[(rho,n,str(run))] = jsonpickle.decode(json_data)

rospy.loginfo("Loaded run stats")


max_mean = 0.0
means = dict()
stds  = dict()
keys = []
for rho in rhos:
    print "Rho:", rho
    means[rho] = []
    stds[rho]  = []
    for n in Ns:
        print "N:", n
        rewards = []
        for run in runs:
            print "Run: ", run
            plan = run_stats[(rho,n,str(run))]
            rewards.append(plan.reward)
        mean = np.mean(rewards)
        means[rho].append(mean)
        std = np.std(rewards)
        stds[rho].append(std)
        print len(means[rho]), len(stds[rho])

tsp_reward =  dict()
# greedy rewards + full TSP solver
tsp_reward['20'] = 1422 
tsp_reward['30'] = 2327

import numpy as np
x = np.asarray([int(n) for n in Ns])
# example data
#x =  np.arange(0.0, len(Ns), 1.0)

y_rho_00 = np.array(means[rhos[0]])
y_rho_05 = np.array(means[rhos[1]])
y_rho_10 = np.array(means[rhos[2]])
y_rho_20 = np.array(means[rhos[3]])
y_rho_25 = np.array(means[rhos[4]])

yerr_rho_00 = tuple(stds[rhos[0]])
yerr_rho_05 = tuple(stds[rhos[1]])
yerr_rho_10 = tuple(stds[rhos[2]])
yerr_rho_20 = tuple(stds[rhos[3]])
yerr_rho_25 = tuple(stds[rhos[4]])

ls = 'dotted'

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


colors = ['blue', 'red', 'yellow', 'green', 'black']
cidx = 0
# standard error bars
plt.errorbar(x, y_rho_00, yerr=yerr_rho_00, marker='^', markersize=20, ls=ls, color=colors[cidx], label=r'$\varrho$ = '+rhos[0])

cidx += 1
plt.errorbar(x, y_rho_05,yerr=yerr_rho_05, marker='>', markersize=20, ls=ls, color=colors[cidx], label=r'$\varrho$ = '+rhos[1])

cidx += 1
plt.errorbar(x, y_rho_10,yerr=yerr_rho_10, marker='<', markersize=20,  ls=ls, color=colors[cidx], label=r'$\varrho$ = '+rhos[2])

cidx += 1
plt.errorbar(x, y_rho_20,yerr=yerr_rho_20, marker='v', markersize=20,  ls=ls, color=colors[cidx], label=r'$\varrho$ = '+rhos[3])

cidx += 1
plt.errorbar(x, y_rho_25,yerr=yerr_rho_25, marker='*', markersize=20,  ls=ls, color=colors[cidx], label=r'$\varrho$ = '+rhos[4])



plt.plot(np.array([tsp_reward[TIME_WINDOW] for i in range(int(Ns[-1])+500 )]), '--k')

# # including upper limits
# uplims = np.zeros(x.shape)
# uplims[[1, 5, 9]] = True
# # including lower limits
# lolims = np.zeros(x.shape)
# lolims[[2, 4, 8]] = True

# # including upper and lower limits
# plt.errorbar(x, y+1.5, marker='o', ms=8, yerr=yerr,
#              lolims=lolims, uplims=uplims, ls=ls, color='magenta')

ax.set_ylim((500, tsp_reward[TIME_WINDOW] + 500))
ax.set_xlim((0, int(Ns[-1]) + 500))
plt.xlabel('Sample size N')
plt.ylabel('Reward')
#plt.title('Average cost by view and parameterisation (rho)')
plt.legend(loc=4)
plt.show()

