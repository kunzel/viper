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

    def keys(self):
        return self.d.keys()

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

    def compute_view_values(self, views, octomap):
        view_values = dict()
        for v in views:
            value = self._robot.evaluate(v, octomap)
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

    def _generate_dependent_value_pmf(self, view_keys, view_keys_values, key_views_map):
        _lambda = 0.5
        pmf = Pmf()

        val_sum = 0
        for view in view_keys:
            vid = view.ID
            val = sum(view.get_values())
            import copy
            view_keys_values[vid] = [copy.deepcopy(list(view.get_keys())), copy.deepcopy(list(view.get_values()))]  
            val_sum += val
            for k in view.get_keys():
                if k not in key_views_map:
                    key_views_map[k] = []
                key_views_map[k].append(vid)

        #print "VIEW -> KEYS VALUES:", view_keys_values
        #print "KEY  -> VIEWS:", key_views_map
        
        Z = val_sum + len(view_keys) * _lambda 
        for view in view_keys:
            vid = view.ID
            val = sum(view.get_values())
            pmf.set(vid, float(val + _lambda) / float(Z))
        return pmf

    def _update_dependent_value_pmf(self, vid, value_pmf, view_keys_values, key_views_map):
        _lambda = 0.5

        #print "UNSET ", vid
        value_pmf.unset(vid)
        #value_pmf.normalize()

        [vkeys, vvals] = view_keys_values[vid]
        # for each voxel in the view

        intersecting_views = []
        for k in vkeys:
            # update all overlapping views
            # print "UPDATE VIEWS ", key_views_map[k] 
            for v in key_views_map[k]:
                [keys,values] = view_keys_values[v]
                key_idx = keys.index(k)
                del keys[key_idx]
                del values[key_idx]
                view_keys_values[v] = [keys,values]
                intersecting_views.append(v)

        val_sum = 0
        for vid in value_pmf.keys():
            [keys, vals] = view_keys_values[vid]
            val = sum(vals)
            val_sum+=val

        Z = val_sum + len(value_pmf.keys()) * _lambda 
        for vid in value_pmf.keys():
            [keys, vals] = view_keys_values[vid]
            val = sum(vals)
            old = value_pmf.prob(vid)

            value_pmf.set(vid, float(val + _lambda) / float(Z))
            #if vid in intersecting_views:
            #    print vid, "!!!! NEW VAL", value_pmf.prob(vid), "OLD VAL", old
            #else:
            #    print vid, "NEW VAL", value_pmf.prob(vid), "OLD VAL", old
        
        #value_pmf.normalize()
        return value_pmf
        
        
    def sample_plans(self, num_of_plans, time_window, rho, views, view_values, view_costs, view_start, view_end):
        plans = []
        for i in range(0, num_of_plans):
            print "Sample plan ", i
            plan = self.sample_plan(str(i), time_window, rho, views, view_values, view_costs, view_start, view_end)
            v_start = view_start.ID
            print "Cost: ", self.calc_plan_cost(plan, view_costs, v_start), "Length: ", len(plan.views)
            plans.append(plan)
        return plans

        
    def sample_plan_1984(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        l = 4
        r = 4.0
    
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v


        plan = Plan(plan_id)
        plan.append(view_start)

        A = list()
        for vid in remaining_views.keys():
            A.append( (math.pow( view_values[vid]/view_costs[view_start.ID][vid] , r), vid) ) 

        A_sorted = sorted(A, reverse=True)
        f = len(A_sorted)
        k = min(l,f)

        pmf = Pmf()
        for i in range(0,k):
            val =  A[i][0]
            vid =  A[i][1]
            pmf.set(vid, val)
        pmf.normalize()

        v_start = view_start.ID
        v_end = view_end.ID
        while len(remaining_views) > 0:
            x = pmf.random()
            plan.append(remaining_views[x])
            # check whether node can be added OR we have to go to v_end (at the moment the starting node)
            current_cost = self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end] # view_costs[x][v_end] NOTE: ADD costs from each node to current_pose in cost matrix
            if current_cost > float(time_window):
                #print "Cost > time_window: ", self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end]
                plan.pop() # remove the
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                return plan

            remaining_views.pop(x)

            # sample without replacement
            A = list()
            for vid in remaining_views.keys():
                A.append( (math.pow( view_values[vid]/view_costs[x][vid] , r), vid) ) 

            A_sorted = sorted(A, reverse=True)
            f = len(A_sorted)
            k = min(l,f)

            pmf = Pmf()
            for i in range(0,k):
                val =  A[i][0]
                vid =  A[i][1]
                pmf.set(vid, val)
            pmf.normalize()            
        return plan

    
    def sample_plan(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        #del remaining_views[view_end.ID]
        #del view_values[view_end.ID]
         # current_view_id = remaining_views.keys()[0] NOW FUNCTION ARGUMENT

        plan = Plan(plan_id)
        plan.append(view_start)
        # pmf for values
        # DETERMINE PMF based on the values of keys
        view_keys_values = dict()
        key_views_map = dict()
        value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)
        #value_pmf = self._generate_value_pmf(view_values)
        
        # TODO: fix unset function, should work even ID is not in the list
        #value_pmf.unset(view_start.ID)
        #value_pmf.unset(view_end.ID)
        value_pmf.normalize()
        
        # pmf for costs
        cost_pmf = self._generate_cost_pmf(view_start.ID, remaining_views.keys(), rho, view_costs)
        # joint dist
        joint = make_joint(value_pmf, cost_pmf)
        joint.normalize()

        v_start = view_start.ID
        v_end = view_end.ID
        while len(remaining_views) > 0:
            x = joint.random()
            plan.append(remaining_views[x])
            # check whether node can be added OR we have to go to v_end (at the moment the starting node)
            current_cost = self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end] # view_costs[x][v_end] NOTE: ADD costs from each node to current_pose in cost matrix
            if current_cost > float(time_window):
                #print "Cost > time_window: ", self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end]
                plan.pop() # remove the
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                return plan

            remaining_views.pop(x)

            # sample without replacement
            # adapt both pmfs: value and cost

            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map
            #value_pmf.unset(x)
            #value_pmf.normalize()
            
            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan

    def calc_plan_cost(self, plan, view_costs, v_start_id):
        cost = 0
        #cost += view_costs[v_start_id][plan.views[0].ID]

        for i in range(len(plan.views)-1):
            cost += view_costs[plan.views[i].ID][plan.views[i+1].ID] 
            
        return cost

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


    
    
    def compute_rewards(self, plans, view_values):
        rewards = dict()
        for p in plans:
            rewards[p.ID] = 0
            for v in p.views:
                rewards[p.ID] += view_values[v.ID]
        return rewards

    def max_reward_plan(self, plan_values):
        return max(plan_values.iteritems(), key=operator.itemgetter(1))[0]
        
    def min_cost_plan(self, plan_values):
        return min(plan_values.iteritems(), key=operator.itemgetter(1))[0]
