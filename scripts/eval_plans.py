#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA
import random
import math

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value



class Vis(object):
    def __init__(self):
        self._server = InteractiveMarkerServer("evaluated_plans")
        self.pubfrustum = rospy.Publisher('frustums', MarkerArray)
        self.marker_id = 0

        
    def _update_cb(self,feedback):
        return

    def visualize_plan(self, plan, plan_vlaues, current_view):
        int_marker = self.create_plan_marker(plan, plan_values, current_view)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

    
    def delete(self, plan):
        self._server.erase(plan.ID)
        self._server.applyChanges()

    def create_frustum_marker(self, markerArray, view, pose, view_values):
        marker1 = Marker()
        marker1.id = self.marker_id
        self.marker_id += 1
        marker1.header.frame_id = "/map"
        marker1.type = marker1.LINE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.color.a = 0.3

        vals = view_values.values()
        max_val = max(vals)
        non_zero_vals = filter(lambda a: a != 0, vals)
        min_val = min(non_zero_vals)
        
        print min_val, max_val, view_values[view.ID]
        
        marker1.color.r = r_func( float((view_values[view.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.g = g_func( float((view_values[view.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.b = b_func( float((view_values[view.ID] - min_val)) /  float((max_val - min_val + 1)))

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position

        points = view.get_frustum()

        marker1.points.append(points[0])
        marker1.points.append(points[1])

        marker1.points.append(points[2])
        marker1.points.append(points[3])
        
        marker1.points.append(points[0])
        marker1.points.append(points[2])

        marker1.points.append(points[1])
        marker1.points.append(points[3])

        marker1.points.append(points[4])
        marker1.points.append(points[5])
        
        marker1.points.append(points[6])
        marker1.points.append(points[7])

        marker1.points.append(points[4])
        marker1.points.append(points[6])
        
        marker1.points.append(points[5])
        marker1.points.append(points[7])

        marker1.points.append(points[0])
        marker1.points.append(points[4])

        marker1.points.append(points[2])
        marker1.points.append(points[6])

        marker1.points.append(points[1])
        marker1.points.append(points[5])

        marker1.points.append(points[3])
        marker1.points.append(points[7])
        
        markerArray.markers.append(marker1)

        
    def create_plan_marker(self, plan, plan_values, current_view):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = plan.ID
        int_marker.description = plan.ID
        pose = Pose()
        #pose.position.x = traj.pose[0]['position']['x']
        #pose.position.y = traj.pose[0]['position']['y']
        int_marker.pose = pose
        
        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.1

        # random.seed(float(plan.ID))
        # val = random.random()
        # line_marker.color.r = r_func(val)
        # line_marker.color.g = g_func(val)
        # line_marker.color.b = b_func(val)
        # line_marker.color.a = 1.0

        line_marker.points = []
        x = current_view.get_robot_pose().position.x
        y = current_view.get_robot_pose().position.y
        z = 0.0 # float(plan.ID) / 10
        p = Point()
        p.x = x - int_marker.pose.position.x  
        p.y = y - int_marker.pose.position.y
        p.z = z - int_marker.pose.position.z
        line_marker.points.append(p)
        for view in plan.views:
            x = view.get_robot_pose().position.x
            y = view.get_robot_pose().position.y
            z = 0.0 # float(plan.ID) / 10
            p = Point()
            p.x = x - int_marker.pose.position.x  
            p.y = y - int_marker.pose.position.y
            p.z = z - int_marker.pose.position.z
            line_marker.points.append(p)

        line_marker.colors = []
        for i in range(len(plan.views)+1):
            color = ColorRGBA()
            val = float(i) / len(plan.views)
            color.r = r_func(val)
            color.g = g_func(val)
            color.b = b_func(val)
            color.a = 1.0
            line_marker.colors.append(color)
                

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker) 
        int_marker.controls.append(control)
        
        return int_marker


rospy.init_node('plan_evaluation')
rospy.loginfo("Started plan evaluation.")

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

INPUT_VIEWS = rospy.get_param('~input_views', 'views_keys.json')

INPUT_FILE = rospy.get_param('~input_file', 'plans.json')
INPUT_FILE_VALUES = rospy.get_param('~input_file_values', 'view_values.json')
INPUT_FILE_COSTS = rospy.get_param('~input_file_costs', 'view_costs.json')
OUTPUT_FILE = rospy.get_param('~output_file', 'plan_values.json')


views = []
with open(INPUT_VIEWS, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))


plans = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    plans = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s plans"  % len(plans))

view_values = dict()
with open(INPUT_FILE_VALUES, "r") as input_file:
    json_data = input_file.read()
    view_values = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view values")

view_costs = dict()
with open(INPUT_FILE_COSTS, "r") as input_file:
    json_data = input_file.read()
    view_costs = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded view costs")

 ###########################################
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState 

try:
    rospy.loginfo("Wait for /robot_pose")
    robotpose_msg = rospy.wait_for_message("/robot_pose", Pose, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /robot_pose")

try:
    rospy.loginfo("Wait for /ptu/state")
    jointstate_msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
except rospy.ROSException, e:
    rospy.logwarn("Failed to get /ptu/state")


 # Add current pose to view_costs (key: '-1')
current_pose = robotpose_msg
rospy.loginfo("Current pose: %s" % current_pose)
current_ptu_state = JointState()
current_ptu_state.name = ['pan', 'tilt']
current_ptu_state.position = [jointstate_msg.position[jointstate_msg.name.index('pan')],jointstate_msg.position[jointstate_msg.name.index('tilt')]]
current_view =  viper.robots.scitos.ScitosView(-1, current_pose, current_ptu_state, None) # ptu pose is not needed for cost calculation
vcosts = dict()
for v in views:
    cost = robot.cost(current_view,v)
    vcosts[v.ID] = cost
    view_costs[v.ID][current_view.ID] = cost  

view_costs[current_view.ID] = vcosts
###########################################

    
planner = ViewPlanner(robot)
plan_values = planner.compute_plan_values(plans, view_values, view_costs)

min_cost_plan_id = planner.min_cost_plan(plan_values)

vis = Vis()
for p in plans:
    print "ID: ", p.ID, " Value: ", plan_values[p.ID]
    vis.visualize_plan(p, plan_values, current_view)
    raw_input()
    vis.delete(p)

# for p in plans:
#     if p.ID == min_cost_plan_id:
#         pids = []
#         for v in p.views:
#             pids.append(v.ID)
#         print "Best plan: ID: ", p.ID, " Value: ", plan_values[p.ID]
#         vis.visualize_plan(p, plan_values, current_view)
#         # frustum marker
#         # call compute_values to calc the frustum
#         view_values = planner.compute_view_values(views)
#         frustum_marker = MarkerArray()    
#         idx = 0
#         for view in views:
#             if view.ID in pids:
#                 val = view_values[view.ID]
#                 print idx, val
#                 if val > 0:
#                     print "Create frustum marker with value", val
#                     if view.get_ptu_pose():
#                         vis.create_frustum_marker(frustum_marker, view, view.get_ptu_pose(), view_values)
#                 idx += 1
#         vis.pubfrustum.publish(frustum_marker)
#         raw_input()
#         vis.delete(p)
#         break


with open(OUTPUT_FILE, "w") as outfile:
    json_data = jsonpickle.encode(plan_values)
    outfile.write(json_data)
    rospy.loginfo("Saved %s plan_values" % len(plan_values.keys()))


rospy.loginfo("Stopped plan evaluation.")
rospy.spin()
