#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.executive import PlanExecutive

rospy.init_node('exec_plan')

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

INPUT_FILE_PLANS_DIR = rospy.get_param('~input_file_plans_dir', '.')
INPUT_FILE_PLANS_NAME = rospy.get_param('~input_file_plans_name', 'plans.json')
INPUT_FILE_PLANS = INPUT_FILE_PLANS_DIR + '/' + INPUT_FILE_PLANS_NAME 
INPUT_FILE_PLAN_VALUES = rospy.get_param('~input_file_plan_values', 'plan_values.json')
OUTPUT_FILE = INPUT_FILE_PLANS_DIR + '/' + INPUT_FILE_PLANS_NAME.split('.json')[0] + '-EXEC_LOG.json' 

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

if p.ID != best_plan_id:
    print "Something bad has happend!"
#best_plan_idx = plans.index(p)

plan_exec = PlanExecutive(robot)
found_objs = []
run_stats = plan_exec.execute(p.views) #  plans[best_plan_idx].views)


run_stats['expected_costs']=  plan_values[best_plan_id]
run_stats['executed_plan'] =  best_plan_id

print "Found objects:"
for o in run_stats['found_objs']:
    print o[1].get('name'), "(", o[0], ")"

with open(OUTPUT_FILE, "w") as outfile:
    json_data = jsonpickle.encode(run_stats)
    outfile.write(json_data)
    rospy.loginfo("Saved %s run stats")
    
rospy.loginfo("Finished plan execution.")
rospy.spin()


