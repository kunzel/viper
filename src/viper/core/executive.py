import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import time

class PlanExecutive(object):

    def __init__(self,robot):
        self._robot = robot 
    
    def execute(self, plan):
        for view in plan:
            self._robot.goto(view)
            self._robot.perform_action(view)

