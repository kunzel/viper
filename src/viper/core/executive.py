import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import time
import rospy

class PlanExecutive(object):

    def __init__(self,robot):
        self._robot = robot 
    
    def execute(self, plan):
        found_objs = []
        start_time = rospy.Time.now()
        for view in plan:
            self._robot.goto(view)
            objs = self._robot.perform_action(view)
            current_time = rospy.Time.now()
            for o in objs:
                time = current_time - start_time
                found_objs.append([time, o])
        return found_objs

