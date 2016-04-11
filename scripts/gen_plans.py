#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState 

rospy.init_node('plan_generation')
rospy.loginfo("Started plan generation.")

import viper.robots.scitos_coverage
robot = viper.robots.scitos_coverage.ScitosRobot()

NUM_OF_PLANS = rospy.get_param('~num_of_plans', 10)
#PLAN_LENGTH = rospy.get_param('~plan_length', 20)
TIME_WINDOW = rospy.get_param('~time_window', 120)
RHO  = rospy.get_param('~rho', 1.0)
BEST_M  = rospy.get_param('~best_m', 10)

ALG  = rospy.get_param('~alg', "VP_S")


INPUT_FILE = rospy.get_param('~input_file', 'view_keys.json')
INPUT_FILE_VALUES = rospy.get_param('~input_file_values', 'view_values.json')
INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', 'view_costs.json')
OUTPUT_FILE = rospy.get_param('~output_file', 'plans.json')
OUTPUT_FILE_BEST = rospy.get_param('~output_file_best', 'best_plan.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))

view_values = dict()
with open(INPUT_FILE_VALUES, "r") as input_file:
    json_data = input_file.read()
    view_values = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view values %s",len(view_values))

view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs %s",len(view_costs))

planner = ViewPlanner(robot)

try:
    rospy.loginfo("Wait for /robot_pose")
    robotpose_msg = rospy.wait_for_message("/robot_pose", Pose, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /robot_pose")

try:
    rospy.loginfo("Wait for /ptu/state")
    jointstate_msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /ptu/state")

    
# Add current pose to view_costs (key: '-1')
current_pose = robotpose_msg
rospy.loginfo("Current pose: %s" % current_pose)
current_ptu_state = JointState()
current_ptu_state.name = ['pan', 'tilt']
current_ptu_state.position = [jointstate_msg.position[jointstate_msg.name.index('pan')],jointstate_msg.position[jointstate_msg.name.index('tilt')]]
current_view =  viper.robots.scitos_coverage.ScitosView("-1", current_pose, current_ptu_state, None) # ptu pose is not needed for cost calculation
vcosts = dict()
for v in views:
    cost = robot.cost(current_view,v)
    vcosts[v.ID] = cost
    view_costs[v.ID][current_view.ID] = cost  

view_costs[current_view.ID] = vcosts
view_costs[current_view.ID][current_view.ID] = 0  

rospy.loginfo("Started plan sampling.")
import time
t_begin = time.time()
if BEST_M <= 0:
    BEST_M = len(views)

if ALG == "VP_S":
    plans = planner.sample_plans_IJCAI(NUM_OF_PLANS, TIME_WINDOW, RHO, BEST_M, views, view_values, view_costs, current_view, current_view)
else:
    print "ERROR: Unkown algorithm!"
t_end = time.time()
planning_time = t_end - t_begin
rospy.loginfo("--- %s seconds ---" % (planning_time))
rospy.loginfo("Stopped plan sampling.")

max_id = -1
max_reward = 0
best_plan = None
for p in plans:
    if p.reward >= max_reward:
        max_reward = p.reward
        max_id = p.ID
        best_plan = p

for p in plans:
    p.planning_time = planning_time
    if p.ID == max_id: #min_cost_plan_id:
        print "Best plan: ID: ", p.ID, " Value: ", p.reward #plan_values[p.ID]

with open(OUTPUT_FILE_BEST, "w") as outfile:
    json_data = jsonpickle.encode(best_plan)
    outfile.write(json_data)
    rospy.loginfo("Saved best plan")

        
with open(OUTPUT_FILE, "w") as outfile:
    json_data = jsonpickle.encode(plans)
    outfile.write(json_data)
    rospy.loginfo("Saved %s plans" % len(plans))

rospy.loginfo("Stopped plan generation.")
#rospy.spin()
