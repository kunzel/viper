import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import random
import math
import time 
import operator
from plan import Plan

class Pmf(object):

    def __init__(self):
        self.d = dict()

    def prob(self, x):
        return self.d[x]  

    def set(self, x, val):
        self.d[x] = val

    def unset(self, x):
        self.d.pop(x)

    def total(self):
        total = sum(self.d.itervalues())
        return total
        
    def normalize(self):
        total = self.total()
        if total == 0.0:
            return total

        for x in self.d:
            self.d[x] /= total

        return total

    def random(self):
        target = random.random()
        total = 0.0
        for x, p in self.d.iteritems():
            total += p
            if total >= target:
                return x


def make_joint(pmf1, pmf2):
    joint = Pmf()
    for vid, p1 in pmf1.d.iteritems():
        p2 = pmf2.prob(vid)
        joint.set(vid, p1 * p2)
    return joint
    

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
            view_values[v.ID] = value
        return view_values

    def compute_view_costs(self, views):
        view_costs = dict()
        i = len(views)
        time_estimate = 0.0
        for v1 in views:
            print("Remaining views: %i --- Remaining time: %s seconds (%s minutes)" % (i, time_estimate,time_estimate/60.0))
            start_time = time.time()
            vcosts = dict()
            for v2 in views:
                # symmetric costs (faster))
                #if v2 in view_costs:
                #    vcosts[v2.ID] = view_costs[v2][v1]
                #else:
                cost = self._robot.cost(v1,v2)
                vcosts[v2.ID] = cost

            view_costs[v1.ID] = vcosts
            i = i - 1
            time_estimate = (time.time() - start_time) * i  
        return view_costs

    def _generate_cost_pmf(self, current_view_id, remaining_view_ids, rho, view_costs):
        pmf = Pmf()
        for vid in remaining_view_ids:
            val =  math.exp(-rho * view_costs[current_view_id][vid])
            pmf.set(vid, val)

        pmf.normalize()
        #print "Pmf cost total:", pmf.total()
        return pmf

    def _generate_value_pmf(self, view_values):
        _lambda = 0.5
        pmf = Pmf()
        Z = sum(view_values.values()) + len(view_values.keys()) * _lambda 
        for vid, val in view_values.iteritems():
            pmf.set(vid, float(val + _lambda) / float(Z))
        #print "Pmf value total:", pmf.total()
        return pmf        
        
    def sample_plans(self, num_of_plans, plan_length, rho, views, view_values, view_costs, current_view_id):
        plans = []
        for i in range(0, num_of_plans):
            print "Sample plan ", i
            plan = self.sample_plan(str(i), plan_length, rho, views, view_values, view_costs, current_view_id)
            plans.append(plan)
        return plans

    def sample_plan(self, plan_id, plan_length, rho, views, view_values, view_costs, current_view_id):
        remaining_views = dict()
        for v in views:
            remaining_views[v.ID] = v

         # current_view_id = remaining_views.keys()[0] NOW FUNCTION ARGUMENT

        plan = Plan(plan_id)
        # pmf for values
        value_pmf = self._generate_value_pmf(view_values)
        # pmf for costs
        cost_pmf = self._generate_cost_pmf(current_view_id, remaining_views.keys(), rho, view_costs)
        # joint dist
        joint = make_joint(value_pmf, cost_pmf)
        joint.normalize()

        for j in range(plan_length):
            x = joint.random()
            plan.append(remaining_views[x])
            remaining_views.pop(x)

            # sample without replacement
            # adapt both pmfs: value and cost
            value_pmf.unset(x)
            value_pmf.normalize()

            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan

    def evaluate_plan(self, plan, view_values, view_costs):
        cost = 0
        prob = 1

        value_pmf = self._generate_value_pmf(view_values)

        current_view = plan.views[0]
        cost += view_costs[current_view.ID][plan.views[0].ID]

        for i in range(len(plan.views)-1):
            prob -= value_pmf.prob(plan.views[i].ID)
            cost += view_costs[plan.views[i].ID][plan.views[i+1].ID] * prob
            
        return cost
    
    def compute_plan_values(self, plans, view_values, view_costs):
        plan_values = dict()
        for p in plans:
            value = self.evaluate_plan(p, view_values, view_costs)
            plan_values[p.ID] = value
        return plan_values

    def min_cost_plan(self, plan_values):
        return min(plan_values.iteritems(), key=operator.itemgetter(1))[0]
