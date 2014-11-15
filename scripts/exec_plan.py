#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.executive import PlanExecutive

rospy.init_node('exec_plan')

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

INPUT_FILE_PLANS = rospy.get_param('~input_file_plans', 'plans.json')
INPUT_FILE_PLAN_VALUES = rospy.get_param('~input_file_plan_values', 'plan_values.json')

plans = []
with open(INPUT_FILE_PLANS, "r") as input_file_plans:
    json_data = input_file_plans.read()
    plans = jsonpickle.decode(json_data)
    print "Loaded plans: ", len(plans)

with open(INPUT_FILE_PLAN_VALUES, "r") as input_file_plan_values:
    json_data = input_file_plan_values.read()
    plan_values = jsonpickle.decode(json_data)
    print "Loaded plan values: ", len(plan_values)    

print "Plan-ID (Value)"
for ID, value in plan_values.iteritems():
    print ID, "(", value, ")"

import operator
best_plan_id = min(plan_values.iteritems(), key=operator.itemgetter(1))[0]
print "Best plan: ", best_plan_id, "(", plan_values[best_plan_id],")"

for p in plans:
    if p.ID == best_plan_id:
        break
else:
    p = None
    
#best_plan_idx = plans.index(p)

plan_exec = PlanExecutive(robot)
found_objs = []
found_objs = plan_exec.execute(p.views) #  plans[best_plan_idx].views)

print "Found objects:"
for o in found_objs:
    print o[1].get('name'), "(", o[0], ")"

# Stats: number of different object
# 'time when obj was found'-graph
    
    
    

rospy.loginfo("Finished plan execution.")
rospy.spin()


