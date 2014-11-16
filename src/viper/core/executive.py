import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import time
import rospy

class PlanExecutive(object):

    def __init__(self,robot):
        self._robot = robot 
    
    def execute(self, plan):
        run_stats = {}
        start_time = rospy.Time.now()
        run_stats['start_time'] = start_time
        run_stats['plan'] = plan
        run_times = []
        found_objs = []
        for view in plan:
            self._robot.goto(view)
            objs = self._robot.perform_action(view)
            current_time = rospy.Time.now()
            run_times.append([current_time, view.ID])
            if objs != None:
                for o in objs:
                    time = current_time - start_time
                    found_objs.append([time, o])
        run_stats['run_times'] = run_times
        run_stats['found_objs'] = found_objs
        end_time = rospy.Time.now() -  start_time
        run_stats['end_time'] = start_time
        return run_stats

