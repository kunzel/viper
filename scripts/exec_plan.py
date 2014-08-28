#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.executive import PlanExecutive

rospy.init_node('exec_plan')

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

FILENAME = rospy.get_param('~input_file', 'views.json')

plan = []
with open(FILENAME, "r") as input_file:
    json_data = input_file.read()
    plan = jsonpickle.decode(json_data)
    print len(plan)

plan_exec = PlanExecutive(robot)

plan_exec.execute(plan)

rospy.loginfo("Finished plan execution.")
rospy.spin()


