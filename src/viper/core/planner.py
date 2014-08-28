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
                # symmetric costs (faster))
                #if v2 in view_costs:
                #    vcosts[v2] = view_costs[v2][v1]
                #else:
                cost = self._robot.cost(v1,v2)
                vcosts[v2] = cost
            view_costs[v1] = vcosts
        return view_costs

    def _generate_cost_pmf(self, view_costs):
        pass

    def _generate_value_pmf(self, view_values):
        pass

    def sample_plans(self, num_of_plans, views, view_values, view_costs):
        plans = []
        for i in range(0, num_of_plans):
            plan = self.sample_plan(views, view_values, view_costs)
            plans.append(plan)
        return plans

    def sample_plan(self, views, view_values, view_costs):

        # recursice method

        return views

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
