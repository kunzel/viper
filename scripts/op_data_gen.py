#!/usr/bin/env python
import rospy
import jsonpickle
from viper.robots.scitos import ScitosView
from geometry_msgs.msg import Pose 
import math

def distance(view1, view2):
    return math.sqrt( math.pow(view1[0]-view2[0],2) + math.pow(view1[1]-view2[1],2))
    
    

rospy.init_node('op_data_gen')

INPUT_FILE = rospy.get_param('~input_file', 'op.txt')
OUTPUT_FILE_VIEWS = rospy.get_param('~output_views', 'views.json')
OUTPUT_FILE_COSTS = rospy.get_param('~output_file_costs', 'view_costs.json')
OUTPUT_FILE_VALUES = rospy.get_param('~output_file_values', 'view_values.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    first_line = input_file.readline()
    problem = first_line.split()
    TMAX = problem[0]
    NUM_PLANS = problem[1]
    lines = input_file.readlines()
    x = dict()
    y = dict()
    rewards = dict()
    for i, l in enumerate(lines):
        v = l.split()
        x[i] = float(v[0])
        y[i] = float(v[1])

        pose = Pose()
        pose.position.x = x[i]
        pose.position.y = y[i]

        view = ScitosView(str(i), pose, None, pose)
        views.append(view)
        
        rewards[str(i)] = int(v[2])

costs = dict()

for i, xpos1 in enumerate(x):
    ypos1 = y[i]
    costs[str(i)] = dict()
    for j, xpos2 in enumerate(x):
        ypos2 = y[j]
        cost = distance((xpos1,ypos1),(xpos2,ypos2))
        costs[str(i)][str(j)] = cost


with open(OUTPUT_FILE_VIEWS, "w") as outfile:
    json_data = jsonpickle.encode(views)
    outfile.write(json_data)

with open(OUTPUT_FILE_VALUES, "w") as outfile:
    json_data = jsonpickle.encode(rewards)
    outfile.write(json_data)

with open(OUTPUT_FILE_COSTS, "w") as outfile:
    json_data = jsonpickle.encode(costs)
    outfile.write(json_data)



       


# # with open(OUTPUT_FILE_COSTS, "w") as outfile:
# #     json_data = jsonpickle.encode(view_costs)
# #     outfile.write(json_data)    

rospy.loginfo("Finished OP data generation.")
rospy.spin()

