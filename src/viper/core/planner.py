import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import itertools
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
        if x in self.d:
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
        return self.keys()[-1]


def make_joint(pmf1, pmf2):
    joint = Pmf()
    for vid, p1 in pmf1.d.iteritems():
        p2 = pmf2.prob(vid)
        joint.set(vid, p1 * p2)
    return joint
    

class ViewPlanner(object):

    def __init__(self,robot):
        self._robot = robot 

    def sample_views_coverage(self, max_num_of_views, min_coverage, octomap, octomap_keys):
        
        # init
        views = []
        coverage = float(0) / len(octomap_keys) 
        num_of_views = 0
        key_val = dict()

        # sample views until ROI is covered
        while coverage < min_coverage and num_of_views < max_num_of_views:
            
            v = self._robot.generate()
            if v != None:
                views.append(v)
            value = self._robot.evaluate(v, octomap)
            for k in v.get_keys():
                if k not in key_val:
                    key_val[k] = v.get_values()[v.get_keys().index(k)]

            # update coverage
            coverage = float(len(key_val.keys())) / len(octomap_keys)
            print('-------------------------------------------')
            print "COVERAGE:", coverage, " #VIEWS:", num_of_views
            print('-------------------------------------------')
            num_of_views += 1

        # give warning if max_number of views was exceeded
        print('===========================================')
        if num_of_views >= max_num_of_views:
            print('WARNING: reached maximum number of views!')
        print('COVERAGE:', coverage)
        print('===========================================')
        return views
        
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
                # import copy
                # [keys,values] = copy.deepcopy(view_keys_values[v])
                # key_idx = keys.index(k)
                # del keys[key_idx]
                # del values[key_idx]
                # view_keys_values[v] = [keys,values]
                # intersecting_views.append(v)
                # import copy
                #
                ##print('WHAT IS HAPPENING!?')
                #print "VIEW -> KEYS VALUES:", view_keys_values
                #print "KEY  -> VIEWS:", key_views_map
                #print "V", v, "K", k
                key_idx = view_keys_values[v][0].index(k)
                view_keys_values[v][0] = [item for i, item in enumerate(view_keys_values[v][0]) if i != key_idx]
                view_keys_values[v][1] = [item for i, item in enumerate(view_keys_values[v][1]) if i != key_idx ]
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
            # if vid in intersecting_views:
            #     print vid, "!!!! NEW VAL", value_pmf.prob(vid), "OLD VAL", old
            # else:
            #     print vid, "NEW VAL", value_pmf.prob(vid), "OLD VAL", old
        
        #value_pmf.normalize()
        return value_pmf
        
        
    def sample_plans(self, num_of_plans, time_window, rho, best_m, views, view_values, view_costs, view_start, view_end):

        # plans = []
        # for i in range(0, num_of_plans):
        #     print "Sample plan ", i
        #     plan = self.sample_plan_DEP_src_best_M(str(i), time_window, rho, best_m, views, view_values, view_costs, view_start, view_end)

                        
        #     v_start = view_start.ID
        #     print "Length: ", len(plan.views), "Cost: ", plan.cost, "Reward: ", plan.reward 
        #     plans.append(tmp_plan)
        # return plans

        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        view_keys_values = dict()
        key_views_map = dict()
        value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)

        A = list()
        while remaining_views:
            x = max(value_pmf.d, key=value_pmf.d.get)# select best view
            A.append(x)
            remaining_views.pop(x)
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
        views_sorted = A
        M = best_m
        considered_views = [view_start.ID] + views_sorted[:M] # consider only the best n views 
        views_subset = []
        for cv in considered_views:
            for v in views:
                if cv == v.ID:
                    views_subset.append(v)
                    

        plans = []
        for i in range(0, num_of_plans):
            print "Sample plan ", i

            #plan = self.sample_plan_DEP_src_best_M(str(i), time_window, rho, best_m, views, view_values, view_costs, view_start, view_end)

            (tmp_cost, plan) = self.sample_plan_DEP_src_best_M2(str(i), time_window, rho, views_subset, view_values, view_costs, view_start, view_end)
            print "FOUND PLAN:", tmp_cost, len(plan.views)
                        
            v_start = view_start.ID
            print "Length: ", len(plan.views), "Cost: ", plan.cost, "Reward: ", plan.reward 
            plans.append(plan)
        return plans


    def baseline_DEP_all_greedy_tsp(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v


        n = 1
        old_cost = 0
        old_plan = Plan(plan_id)
        #if n == n:
        while n < len(views_sorted):

            considered_views = [view_start.ID] + remaining_views.keys()

            print views_sorted[:n]
            print considered_views
            
            (tmp_cost, tmp_plan) = self.solve_tsp_greedy(plan_id, considered_views, views, view_values, view_costs, view_start, view_end) 
            print "RETURNED", tmp_cost, len(tmp_plan.views)
            
            if tmp_cost > float(time_window):
                return old_plan
            else:
                (old_cost, old_plan) = (tmp_cost, tmp_plan)
            # consider one view more 
            n += 1

        # No plan was found for time_window! Return plan with v_start/v_end: here the same
        return old_plan

    def baseline_DEP_greedy_tsp(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        view_keys_values = dict()
        key_views_map = dict()
        value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)

        A = list()
        while remaining_views:
            x = max(value_pmf.d, key=value_pmf.d.get)# select best view
            A.append(x)
            remaining_views.pop(x)
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
            
        views_sorted = A

        n = 1
        old_cost = 0
        old_plan = Plan(plan_id)
        #if n == n:
        while n < len(views_sorted):

            considered_views = [view_start.ID] + views_sorted[:n] # consider only the best n views 

            print views_sorted[:n]
            print considered_views
            
            (tmp_cost, tmp_plan) = self.solve_tsp_greedy(plan_id, considered_views, views, view_values, view_costs, view_start, view_end) 
            print "RETURNED", tmp_cost, len(tmp_plan.views)
            
            if tmp_cost > float(time_window):
                return old_plan
            else:
                (old_cost, old_plan) = (tmp_cost, tmp_plan)
            # consider one view more 
            n += 1

        # No plan was found for time_window! Return plan with v_start/v_end: here the same
        return old_plan

        
        
    def baseline_DEP_tsp(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        view_keys_values = dict()
        key_views_map = dict()
        value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)

        A = list()
        while remaining_views:
            x = max(value_pmf.d, key=value_pmf.d.get)# select best view
            A.append(x)
            remaining_views.pop(x)
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
            
        views_sorted = A

        n = 1
        old_cost = 0
        old_plan = Plan(plan_id)
        #if n == n:
        while n < len(views_sorted):

            considered_views = [view_start.ID] + views_sorted[:n] # consider only the best n views 

            print views_sorted[:n]
            print considered_views
            
            (tmp_cost, tmp_plan) = self.solve_tsp(plan_id, considered_views, views, view_values, view_costs, view_start, view_end) 
            print "RETURNED", tmp_cost, len(tmp_plan.views)
            
            if tmp_cost > float(time_window):
                return old_plan
            else:
                (old_cost, old_plan) = (tmp_cost, tmp_plan)
            # consider one view more 
            n += 1

        # No plan was found for time_window! Return plan with v_start/v_end: here the same
        return old_plan
        
    def baseline_IND_tsp(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
    
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        A = list()
        for vid in remaining_views.keys():
            A.append( (view_values[vid], vid) ) 

        A_sorted = sorted(A, reverse=True)
        views_sorted = [v for (val,v) in A_sorted]

        n = 1
        old_cost = 0
        old_plan = Plan(plan_id)
        #if n == n:
        while n < len(A_sorted):

            considered_views = [view_start.ID] + views_sorted[:n] # consider only the best n views 

            print A_sorted[:n]
            print considered_views
            
            (tmp_cost, tmp_plan) = self.solve_tsp(plan_id, considered_views, views, view_values, view_costs, view_start, view_end) 
            print "RETURNED", tmp_cost, len(tmp_plan.views)
            
            if tmp_cost > float(time_window):
                return old_plan
            else:
                (old_cost, old_plan) = (tmp_cost, tmp_plan)
            # consider one view more 
            n += 1

        # No plan was found for time_window! Return plan with v_start/v_end: here the same
        return old_plan

    def solve_tsp_greedy(self, plan_id, considered_views, views, view_values, view_costs, view_start, view_end):

        # for all considered views
        # find the next best view greedily
        # and add it to the path
        
        plan = Plan(plan_id)
        plan.append(view_start)
        current_view_ID = view_start.ID
        considered_views.pop(considered_views.index(view_start.ID))

        cost = 0 
        while len(considered_views) > 0:
            # find best next view
            import sys
            min_cost = sys.float_info.max
            view_id = None
            for v in considered_views:
                if view_costs[current_view_ID][v] < min_cost:
                    min_cost =  view_costs[current_view_ID][v]
                    view_id = v
                    
            for v in views:
                if view_id == v.ID:
                    plan.append(v)
                    considered_views.pop(considered_views.index(view_id))
                    cost += min_cost

        last_view_ID = plan.views[-1].ID
        plan.append(view_end)
        plan.cost = cost + view_costs[last_view_ID][view_end.ID]
        # print "Plan:"
        # for v in plan.views:
        #     print v.ID
	return (plan.cost, plan)
        
    def solve_tsp(self, plan_id, considered_views, views, view_values, view_costs, view_start, view_end):
        tour = []
        vid = ['XXX'] + considered_views
        numViews = len(considered_views)
        
	vertices = range(1, numViews +1)
	A = {}
	
	# Do this here, as m starts from 2
	A[tuple([1])] = {}
	A[tuple([1])][1] = 0
	
	for m in range(2, numViews + 1):	# m = subproblem size (cardinality of S) 
		print("M =", m)
		combos = itertools.combinations(vertices, m)
		
		# if m >= 4:
		# 	toDel = itertools.combinations(vertices, m - 2)
		# 	for key in toDel:
		# 		if 1 in key:
		# 			del A[key]
			
		for S in combos:			# Take all possible subsets of size m
			# POSSIBLE OPTIMIZATION HERE:
				# Map all possible subsets as bitmap strings and just set cities which are included in the subset to 1.
				# This would reduce memory consumption a lot and speed up the loop operations.
				# E.G: 0000000000000000000000111 (single string) as a dictionary key instead of the tuple of n integers.
			if 1 not in S:			# S has to contain 1
				continue
			# Set up base cases for this m
			A[S] = {}
			A[S][1] = float("inf")
			
			for j in S:				# Iterate through all j in S where j != 1 (j = 1 has been taken care of in base cases)
				if j == 1:
					continue
				minASj = float("inf")
				for k in S:			# Iterate through all k in S where k != j and find best k for the subproblem
					if k == j:
						continue
					sNew = list(S)
					sNew.remove(j)
					if A[tuple(sNew)][k] + view_costs[vid[k]][vid[j]] < minASj:
						minASj = A[tuple(sNew)][k] + view_costs[vid[k]][vid[j]]
				A[S][j] = minASj

	# Find the shortest tour out of all narrowed candidates
	minTour = float("inf")
	for j in range(2, numViews + 1):
		if A[tuple(range(1, numViews + 1))][j] + view_costs[vid[j]][vid[1]] < minTour:	# Just compare the total distances including the final hop
			print(tuple(range(1, numViews + 1)))
			minTour = A[tuple(range(1, numViews + 1))][j] + view_costs[vid[j]][vid[1]]
                        minJ = j


        # RECONSTRUCT TOUR
        #print "min J:", minJ
        minJs = [minJ]
        subs = [i for i in range(1, numViews + 1) if i not in minJs]
        tour.append(1) # end
        tour.append(minJ)
        #print "SUBS:", subs, "A[SUBS]", A[tuple(subs)]
        while subs:
            minJ = min(A[tuple(subs)].items(), key=lambda x: x[1])[0]
            tour.append(minJ)
            minJs.append(minJ)
            subs = [i for i in range(1, numViews + 1) if i not in minJs]

        rtour = list(reversed(tour))
        print "TOUR:", rtour
        # VERIFY TOUR COSTS!!!
        # for i in range(0,len(rtour)-1):
        #     cost += view_costs[vid[rtour[i]]][vid[rtour[i+1]]]
        # print "COST:", cost
        # print "MINTOUR: ", minTour

        plan = Plan(plan_id)
        plan.append(view_start)
        for node in rtour:
            view_id = vid[node]
            for v in views:
                if view_id == v.ID:
                    plan.append(v)
        plan.append(view_end)
        plan.cost = minTour
	return (minTour, plan)
        
        
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
        k = f # min(l,f)

        pmf = Pmf()
        for i in range(0,k):
            val =  A[i][0]
            vid =  A[i][1]
            pmf.set(vid, val)
        pmf.normalize()

        v_start = view_start.ID
        v_end = view_end.ID
        while len(remaining_views) > 0:
            #print pmf.d
            x = pmf.random()
            #print x
            plan.append(remaining_views[x])
            # check whether node can be added OR we have to go to v_end (at the moment the starting node)
            current_cost = self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end] # view_costs[x][v_end] NOTE: ADD costs from each node to current_pose in cost matrix
            if current_cost > float(time_window):
                #print "Cost > time_window: ", self.calc_plan_cost(plan, view_costs, v_start) + view_costs[x][v_end]
                plan.pop() # remove the
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            remaining_views.pop(x)

            # sample without replacement
            A = list()
            for vid in remaining_views.keys():
                A.append( (math.pow( view_values[vid]/view_costs[x][vid] , r), vid) ) 

            A_sorted = sorted(A, reverse=True)
            f = len(A_sorted)
            k = f # min(l,f)

            pmf = Pmf()
            for i in range(0,k):
                val =  A[i][0]
                vid =  A[i][1]
                pmf.set(vid, val)
            pmf.normalize()            
        return plan

    def sample_plan_IND_src(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v


        plan = Plan(plan_id)
        plan.append(view_start)
        # pmf for values
        # DETERMINE PMF based on the values of keys
        view_keys_values = dict()
        key_views_map = dict()
        dep_value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)

        value_pmf = self._generate_value_pmf(view_values)
        
        # TODO: fix unset function, should work even ID is not in the list
        value_pmf.unset(view_start.ID)
        value_pmf.unset(view_end.ID)
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            remaining_views.pop(x)
            plan.reward += sum(view_keys_values[x][1])
            plan.ind_reward += view_values[x]
                                        
            # sample without replacement
            # adapt both pmfs: value and cost

            dep_value_pmf = self._update_dependent_value_pmf(x, dep_value_pmf, view_keys_values, key_views_map)

            value_pmf.unset(x)
            value_pmf.normalize()
            
            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan
    
    def sample_plan_DEP_src(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        #del remaining_views[view_end.ID]
        #del view_values[view_end.ID]
         # current_view_id = remaining_views.keys()[0] NOW FUNCTION ARGUMENT

        reward = 0
        keys = dict()
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            #print "SELECTED", x, view_keys_values[x][0]
            remaining_views.pop(x)
            for k in view_keys_values[x][0]:
                if k not in keys:
                    keys[k] = True
                    reward += 1
                else:
                    print "FATAL ERROR: VID", x, "key", k, key_views_map[k]

            plan.reward += sum(view_keys_values[x][1])
            if reward != plan.reward:
                print "FATAL ERROR: reward", reward, "plan.reward", plan.reward
            # sample without replacement
            # adapt both pmfs: value and cost

            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)

            #value_pmf.unset(x)
            #value_pmf.normalize()
            
            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan

    def sample_plan_DEP_src_best_M(self, plan_id, time_window, rho, best_m, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v

        view_keys_values = dict()
        key_views_map = dict()
        value_pmf = self._generate_dependent_value_pmf(views, view_keys_values, key_views_map)

        A = list()
        while remaining_views:
            x = max(value_pmf.d, key=value_pmf.d.get)# select best view
            A.append(x)
            remaining_views.pop(x)
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
            
        views_sorted = A


        M = best_m
        considered_views = [view_start.ID] + views_sorted[:M] # consider only the best n views 
        views_subset = []
        for cv in considered_views:
            for v in views:
                if cv == v.ID:
                    views_subset.append(v)
                    
        (tmp_cost, tmp_plan) = self.sample_plan_DEP_src_best_M2(plan_id, time_window, rho, views_subset, view_values, view_costs, view_start, view_end)
        print "FOUND PLAN:", tmp_cost, len(tmp_plan.views)
        return tmp_plan

        # n = 1
        # old_cost = 0
        # old_plan = Plan(plan_id)
        # #if n == n:
        # while n < len(views_sorted):

        #     considered_views = [view_start.ID] + views_sorted[:n] # consider only the best n views 
        #     views_subset = []
        #     for cv in considered_views:
        #         for v in views:
        #             if cv == v.ID:
        #                 views_subset.append(v)
            
        #     #print views_sorted[:n]
        #     #print considered_views
            
        #     (tmp_cost, tmp_plan) = self.sample_plan_DEP_src_best_M2(plan_id, time_window, rho, views_subset, view_values, view_costs, view_start, view_end)
        #     print "RETURNED", tmp_cost, len(tmp_plan.views)
            
        #     if tmp_cost > float(time_window):
        #         return old_plan
        #     else:
        #         (old_cost, old_plan) = (tmp_cost, tmp_plan)
        #     # consider one view more 
        #     n += 1

        # # No plan was found for time_window! Return plan with v_start/v_end: here the same
        # return old_plan


                
    def sample_plan_DEP_src_best_M2(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
        remaining_views = dict()
        for v in views:
            if v.ID != view_start.ID and v.ID != view_end.ID: 
                remaining_views[v.ID] = v
        #del remaining_views[view_end.ID]
        #del view_values[view_end.ID]
         # current_view_id = remaining_views.keys()[0] NOW FUNCTION ARGUMENT

        reward = 0
        keys = dict()
        plan = Plan(plan_id)
        plan.append(view_start)
        plan.cost = 0
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return (plan.cost, plan)

            #print "SELECTED", x, view_keys_values[x][0]
            remaining_views.pop(x)
            for k in view_keys_values[x][0]:
                if k not in keys:
                    keys[k] = True
                    reward += 1
                else:
                    print "FATAL ERROR: VID", x, "key", k, key_views_map[k]

            plan.reward += sum(view_keys_values[x][1])
            if reward != plan.reward:
                print "FATAL ERROR: reward", reward, "plan.reward", plan.reward
            # sample without replacement
            # adapt both pmfs: value and cost

            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)

            #value_pmf.unset(x)
            #value_pmf.normalize()
            
            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return (plan.cost, plan)

    def sample_plan_DEP_sc(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
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
        #value_pmf.normalize()
        
        # pmf for costs
        cost_pmf = self._generate_cost_pmf(view_start.ID, remaining_views.keys(), rho, view_costs)
        # joint dist
        joint = cost_pmf #make_joint(value_pmf, cost_pmf)
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            remaining_views.pop(x)
            plan.reward += sum(view_keys_values[x][1])

            # sample without replacement
            # adapt both pmfs: value and cost
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
            #value_pmf.unset(x)
            #value_pmf.normalize()
            
            cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = cost_pmf #make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan

    def sample_plan_DEP_sr(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
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
        #cost_pmf = self._generate_cost_pmf(view_start.ID, remaining_views.keys(), rho, view_costs)
        # joint dist
        joint = value_pmf #make_joint(value_pmf, cost_pmf)
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            remaining_views.pop(x)
            plan.reward += sum(view_keys_values[x][1])

            # sample without replacement
            # adapt both pmfs: value and cost
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
            #value_pmf.unset(x)
            #value_pmf.normalize()
            
            #cost_pmf = self._generate_cost_pmf(x, remaining_views.keys(), rho, view_costs)
            # re-generate joint dist
            joint = value_pmf #make_joint(value_pmf, cost_pmf)
            joint.normalize()
        return plan

    def sample_plan_DEP_drc(self, plan_id, time_window, rho, views, view_values, view_costs, view_start, view_end):
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
                plan.pop() # remove x
                #print "Cost OK: ", self.calc_plan_cost(plan, view_costs, v_start)
                plan.append(view_end)
                plan.cost = self.calc_plan_cost(plan, view_costs, v_start)
                return plan

            remaining_views.pop(x)
            plan.reward += sum(view_keys_values[x][1])

            # sample without replacement
            # adapt both pmfs: value and cost
            value_pmf = self._update_dependent_value_pmf(x, value_pmf, view_keys_values, key_views_map)
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
