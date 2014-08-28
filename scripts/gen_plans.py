#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner

rospy.init_node('plan_generation')
rospy.loginfo("Started plan generation.")

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

NUM_OF_PLANS = rospy.get_param('~num_of_plans', 100)
PLAN_LENGTH = rospy.get_param('~plan_length', 20)
RHO  = rospy.get_param('~rho', 1.0)


INPUT_FILE = rospy.get_param('~input_file', 'views.json')
INPUT_FILE_VALUES = rospy.get_param('~input_file_values', 'view_values.json')
INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', 'view_costs.json')
OUTPUT_FILE = rospy.get_param('~output_file', 'plans.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))

view_values = dict()
with open(INPUT_FILE_VALUES, "r") as input_file:
    json_data = input_file.read()
    view_values = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view values")

view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs")

planner = ViewPlanner(robot)

rospy.loginfo("Started plan sampling.")
plans = planner.sample_plans(NUM_OF_PLANS, PLAN_LENGTH, RHO, views, view_values, view_costs)
rospy.loginfo("Stopped plan sampling.")

with open(OUTPUT_FILE, "w") as outfile:
    json_data = jsonpickle.encode(plans)
    outfile.write(json_data)
    rospy.loginfo("Saved %s plans" % len(plans))

rospy.loginfo("Stopped plan generation.")

