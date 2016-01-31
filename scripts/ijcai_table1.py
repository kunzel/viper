#!/usr/bin/env python
import rospy
import jsonpickle
import sys
from tabulate import tabulate

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState 

import viper.robots.scitos_coverage
robot = viper.robots.scitos_coverage.ScitosRobot()

rospy.init_node('table')
    
INPUT_FILE_DIR = rospy.get_param('~input_file_dir', '.')

# # alg = "FINALBESTM"
# # MAX_REWARD = 2637
# # OLD G4S!!!!
# alg = "G4SBESTM"
# MAX_REWARD = 11128
# runs = [0]
# rhos = ['1.0'] #['0.0,','0.5','1,0','2.0']
# # variables that effect planning time
# N = [5, 10, 20, 30, 40, 50, 100, 250, 500, 1000] #, 2000] 
# TIME =  [30, 60, 120, 180, 240, 300, 600]
# #time_colors = [('r', '^'), ('g', '>'), ('b', 'v'), ('y', '<'),('m', '.'),('c', 'o'),('w','+'),('k','*')]
# BEST_M = [5, 10, 15, 20, 30, 50, 100, 125, -1] #[5, 10, 15, 20, 30, 50, 100, 125, 165]
# TWINDOW = [60, 120, 180, 240, 300, 600]

INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', '../view_costs.json')
INPUT_FILE = rospy.get_param('~input_file', '../view_keys.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))


view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs")

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
    
#Irlab 
MAX_REWARD = 2637

#G4s new
#MAX_REWARD = 10498


print "Plan analysis started:"
for f in sys.argv[1:]:
    table = []
    plan = None 
    with open(f, "r") as input_file:
        json_data = input_file.read()
        plan = jsonpickle.decode(json_data)


        # calc plan costs
        costs = 0
        for i in range(1,len(plan.views)):
            costs +=  view_costs[plan.views[i-1].ID][plan.views[i].ID]
        
        keys = dict()
        reward = 0
        vids = []
        for v in plan.views:
            vids.append(v.ID)
            for k in v.get_keys():
                if k not in keys:
                    keys[k] = True
                    reward += 1

            
        #print  f, "-- Reward:", reward, "(plan.reward:", plan.reward ,") Cost", plan.cost, costs, "Length:", len(plan.views) #, vids
        #print ""

        # TABLE 1
        import math
        #table.append([ f, float(reward) / MAX_REWARD,'&', math.floor(costs),'&', math.floor(plan.planning_time), '&', len(plan.views),'\\\\'])

        # TABLE 2
        table.append([ f, float(reward) / MAX_REWARD,'&', len(plan.views), '&'])

    # TABLE 1
    #print tabulate(table, headers=['CONF', 'R','&', 'T_E', '&', 'T_P', '&','|s|','\\\\'],tablefmt='rst')
    # TABLE 2
    print tabulate(table, headers=['CONF', 'R','&', '|s|', '&'''],tablefmt='rst')


print "Plan analysis finished."
