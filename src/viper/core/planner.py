import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot

class ViewPlanner(object):

    def __init__(self,robot):
        self._robot = robot 

    def sample_views(self, num_of_views):
        views = []
        for i in range(0,num_of_views):
            view = self._robot.generate()
            if view != None:
                views.append(view)
        return views

    def compute_view_values(self, views):
        view_values = dict()
        for v in views:
            value = self._robot.evaluate(v)
            view_values[v] = value
        return view_values

    def compute_view_costs(self, views):
        view_costs = dict()
        for v1 in views:
            vcosts = dict()
            for v2 in views:
                cost = self._robot.cost(v1,v2)
                vcost[v2] = cost
            view_costs[v1] = vcost
        return view_costs

    def _generate_cost_pmf(self, view_costs):
        pass

    def _generate_value_pmf(self, view_values):
        pass

    def sample_plans(self, num_of_plans):
        plans = []
        for i in range(0, num_of_plans):
            plan = self.sample_plan()
            plans.append(plan)
        return plans

    def sample_plan():
        pass

    def evaluate_plan():
        pass

    def compute_plan_values(self, paths):
        plan_values = dict()
        for p in plans:
            value = self.evaluate_plan(p)
            plan_values[p] = value
        return plan_values

    def select_best_plan(self, plan_values):
        pass
