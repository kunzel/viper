#!/usr/bin/env python
import jsonpickle
import sys


print "Plan analysis started:"
for f in sys.argv[1:]:
    plan = None 
    with open(f, "r") as input_file:
        json_data = input_file.read()
        plan = jsonpickle.decode(json_data)

        keys = dict()
        reward = 0
        vids = []
        for v in plan.views:
            vids.append(v.ID)
            for k in v.get_keys():
                if k not in keys:
                    keys[k] = True
                    reward += 1

            
        print  f, "-- Reward:", reward, "(plan.reward:", plan.reward ,") Cost", plan.cost, "Length:", len(plan.views) #, vids
        #print ""

        

print "Plan analysis finished."
