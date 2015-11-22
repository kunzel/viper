#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState 

rospy.init_node('plan_generation')
rospy.loginfo("Started plan generation.")

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

NUM_OF_PLANS = rospy.get_param('~num_of_plans', 10)
TIME_WINDOW = rospy.get_param('~time_window', 120)
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


v_start = None
v_end = None
for v in views:
    if v.ID == str(0):
        v_start = v
    if v.ID == str(1):
        v_end = v

if v_start == None or v_end == None:
    print "ERROR: Could not find v_start and/or v_end!!!"
    
planner = ViewPlanner(robot)

rospy.loginfo("Started plan sampling.")
plans = planner.sample_plans(NUM_OF_PLANS, TIME_WINDOW, RHO, views, view_values, view_costs, v_start, v_end)
rospy.loginfo("Stopped plan sampling.")

plan_values = planner.compute_rewards(plans, view_values)
max_reward_plan_id = planner.max_reward_plan(plan_values)

for p in plans:
    if p.ID == max_reward_plan_id:
        best_plan = p
        break

        
plan_cost =  planner.calc_plan_cost(best_plan, view_costs, best_plan.views[0].ID)

print "Best plan: ID=", max_reward_plan_id, " Reward=", plan_values[max_reward_plan_id], " Cost:", plan_cost    

reward = 0
cost = 0
for i, v in enumerate(best_plan.views):
    print v.ID, " - ", view_values[v.ID]
    reward += view_values[v.ID]
    if v.ID != "1":
        cost += view_costs[best_plan.views[i].ID][best_plan.views[i+1].ID]
                           
print "Total reward: ", reward, " total costs: ", cost

with open(OUTPUT_FILE, "w") as outfile:
    json_data = jsonpickle.encode(plans)
    outfile.write(json_data)
    rospy.loginfo("Saved %s plans" % len(plans))

rospy.loginfo("Stopped plan generation.")
rospy.spin()
