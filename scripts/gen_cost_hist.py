#!/usr/bin/env python

from matplotlib.pylab import *
import rospy
import jsonpickle

rospy.init_node('gen_cost_hist')
rospy.loginfo("Started histogramm generation.")

INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', 'view_costs.json')

view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs")

mat = []
for v1 in view_costs.keys():
    row = []
    for v2 in view_costs.keys():
        cost = view_costs[v1][v2]
        row.append(cost)
    mat.append(row)
        
#print mat
#mat = [[i for i in range(500)] for j in range(500)]

matshow(mat) #cmap=cm.gray)
colorbar()
show()
