#!/usr/bin/env python
import jsonpickle



INPUT_VIEWS = './view_keys.json'
INPUT_VIEW_VALUES = './view_values.json'

views = []
with open(INPUT_VIEWS, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    print "Loaded %s views"  % len(views)

view_values = None 
with open(INPUT_VIEW_VALUES, "r") as input_file:
    json_data = input_file.read()
    view_values = jsonpickle.decode(json_data)


keys = dict()

reward = 0
value = 0
for v in views:
    value += view_values[v.ID]
    for k in v.get_keys():
        if k not in keys:
            keys[k] = True
            reward += 1

print "Max Reward:", reward
print "Value:", value
