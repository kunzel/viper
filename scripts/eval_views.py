#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose


class Vis(object):

    def __init__(self):
        self.pubmarker = rospy.Publisher('evaluated_views', MarkerArray)
        self.pubfrustum = rospy.Publisher('frustums', MarkerArray)
        self.pubcost =  rospy.Publisher('costs', MarkerArray)
        self.marker_id = 0

    def create_cost_marker(self, markerArray, view1, view2, view_costs):

        marker1 = Marker()
        marker1.id = self.marker_id
        self.marker_id += 1
        marker1.header.frame_id = "/map"
        marker1.type = marker1.LINE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 0.001
        marker1.color.a = 0.3

        costs = []
        row_costs = view_costs.values()
        for row in row_costs:
            costs.append(row.values())
        flatted_costs =  [val for sublist in costs for val in sublist]
        
        max_val = max(flatted_costs)
        non_zero_vals = filter(lambda a: a != 0, flatted_costs)
        min_val = min(non_zero_vals)
        
        marker1.color.r = r_func( float((view_costs[view1.ID][view2.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.g = g_func( float((view_costs[view1.ID][view2.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.b = b_func( float((view_costs[view1.ID][view2.ID] - min_val)) /  float((max_val - min_val + 1)))

        pose = Pose() #view1.get_robot_pose()
        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position

        point1 = Point()
        point1.x =  view1.get_ptu_pose().position.x 
        point1.y =  view1.get_ptu_pose().position.y
        point1.z =  view1.get_ptu_pose().position.z 
        point2 = Point()
        point2.x =  view2.get_ptu_pose().position.x
        point2.y =  view2.get_ptu_pose().position.y
        point2.z =  view2.get_ptu_pose().position.z

        marker1.points = []
        marker1.points.append(point1)
        marker1.points.append(point2)
        
        markerArray.markers.append(marker1)


        
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


    def create_marker(self, markerArray, view, pose, view_values):
        marker1 = Marker()
        marker1.id = self.marker_id
        self.marker_id += 1
        marker1.header.frame_id = "/map"
        marker1.type = marker1.TRIANGLE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 1
        marker1.scale.y = 1
        marker1.scale.z = 2
        marker1.color.a = 0.25

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
        marker1.points = [Point(0,0,0.01),Point(2.5,-1.39,0.01),Point(2.5,1.39,0.01)]
        
        markerArray.markers.append(marker1)

    def delete_markers(self):
        markerArray = MarkerArray()
        for i in range(0,self.marker_len):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = i
            marker.action = marker.DELETE
            markerArray.markers.append(marker)
        self.pubmarker.publish(markerArray)


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


def hist(views):
    max_val = 0
    data = []
    h = dict()
    for v in views:
        val = sum(v.get_values())
        data.append(val)
        if val not in h:
            h[val] = 0
        h[val] +=1
        if val > max_val:
            max_val = val

    # data = [0] * (max_val+1)
    # for val in h.keys():
    #     data[val] = h[val]

    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import matplotlib.path as path

    fig, ax = plt.subplots()
    
    # histogram our data with numpy
    #data = np.random.randn(1000)
    n, bins = np.histogram(data, len(data))
    
    # get the corners of the rectangles for the histogram
    left = np.array(bins[:-1])
    right = np.array(bins[1:])
    bottom = np.zeros(len(left))
    top = bottom + n


    # we need a (numrects x numsides x 2) numpy array for the path helper
    # function to build a compound path
    XY = np.array([[left, left, right, right], [bottom, top, top, bottom]]).T
    
    # get the Path object
    barpath = path.Path.make_compound_path_from_polys(XY)
    
    # make a patch out of it
    patch = patches.PathPatch(
        barpath, facecolor='blue', edgecolor='gray', alpha=0.8)
    ax.add_patch(patch)

    # update the view limits
    ax.set_xlim(left[0], right[-1])
    ax.set_ylim(bottom.min(), top.max())

    plt.show()

    

    

vis = Vis()
robot_poses_pub = rospy.Publisher('robot_poses', PoseArray)

rospy.init_node('view_evaluation')

import viper.robots.scitos_coverage
robot = viper.robots.scitos_coverage.ScitosRobot()

INPUT_FILE = rospy.get_param('~input_file',   'views.json')
OUTPUT_FILE_COSTS = rospy.get_param('~output_file_costs', 'view_costs.json')
OUTPUT_FILE_VALUES = rospy.get_param('~output_file_values', 'view_values.json')
OUTPUT_FILE_VIEW_KEYS = rospy.get_param('~output_file_view_keys', 'view_keys.json')

views = []
with open(INPUT_FILE, "r") as input_file:
    json_data = input_file.read()
    views = jsonpickle.decode(json_data)
    rospy.loginfo("Loaded %s views"  % len(views))

from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap, GetOctomapRequest

octomap = Octomap()
# # #octomap_service_name = '/Semantic_map_publisher_node/SemanticMapPublisher/ObservationOctomapService'
# # rospy.loginfo("Waiting for octomap service")
# # octomap_service_name = '/octomap_full'
# # rospy.wait_for_service(octomap_service_name)
# # rospy.loginfo("Done")
# # try:
# #     octomap_service = rospy.ServiceProxy(octomap_service_name, GetOctomap)
# #     req = GetOctomapRequest()
# #     rospy.loginfo("Requesting octomap from semantic map service")
# #     res = octomap_service(req)
# #     octomap = res.map
# #     rospy.loginfo("Received octomap: size:%s resolution:%s", len(octomap.data), octomap.resolution)

# # except rospy.ServiceException, e:
# #     rospy.logerr("Service call failed: %s"%e)

from viper.srv import GetKeys, GetKeysRequest
rospy.loginfo("Waiting for octomap get-keys service")
service_name = '/get_keys'
rospy.wait_for_service(service_name)
rospy.loginfo("Done")
try:
    service = rospy.ServiceProxy(service_name, GetKeys)
    req = GetKeysRequest()
    req.octomap = octomap
    rospy.loginfo("Requesting keys from octomap get-keys service")
    res = service(req)
    octomap_keys = res.keys
    rospy.loginfo("Received octomap_keys: size:%s", len(octomap_keys))
    #print "OCTOMAP KEYS", octomap_keys
    
except rospy.ServiceException, e:
    rospy.logerr("Service call failed: %s"%e)


#hist(views)
# FILTER VIEWS

covered_keys = dict() 
for v in views:
    for k in v.get_keys():
        if k not in covered_keys:
            covered_keys[k] = 0
        covered_keys[k] += v.get_values()[v.get_keys().index(k)]

coverage = float(len(covered_keys.keys()))/float(len(octomap_keys))
min_coverage = 0.95
x = 0
covered_keys = dict()
filtered_views = []
import copy
while coverage > min_coverage:
    old_covered_keys = copy.deepcopy(covered_keys)
    covered_keys = dict()
    old_filtered_views = copy.deepcopy(filtered_views)
    filtered_views = []
    for v in views:
        val = sum(v.get_values()) 
        if  val > x:
            filtered_views.append(v)
            for k in v.get_keys():
                if k not in covered_keys:
                    covered_keys[k] = 0
                covered_keys[k] += v.get_values()[v.get_keys().index(k)]
    x += 1
    coverage =  float(len(covered_keys.keys()))/float(len(octomap_keys))
    
print "#FILTERED VIEWS:", len(old_filtered_views), "Coverage: ", float(len(old_covered_keys.keys()))/float(len(octomap_keys)), "(Views with values 0<=value<=X have been filtered out) X:", x-1  

hist(old_filtered_views)

# save unfiltered views
with open(INPUT_FILE + "-UNFILTERED", "w") as outfile:
    json_data = jsonpickle.encode(views)
    outfile.write(json_data)

views = old_filtered_views

view_values = dict()
for v in views:
    value = sum(v.get_values())
    view_values[v.ID] = value
planner = ViewPlanner(robot)
# view_values = planner.compute_view_values(views, octomap)

robot_poses  = PoseArray()
robot_poses.header.frame_id = '/map'
robot_poses.poses = []
for v in views:
    robot_poses.poses.append(v.get_robot_pose())
robot_poses_pub.publish(robot_poses)

print "COMPUTE COSTS..."
#### COSTS #######################################
view_costs = []
view_costs = planner.compute_view_costs(views)

#################################################

# # triangle marker
# markerArray = MarkerArray()    
# idx = 0
# for view in views:
#     val = view_values[view.ID]
#     print idx, val
#     if val > 0:
#         print "Create triangle marker with value", val
#         vis.create_marker(markerArray, view, view.get_ptu_pose(), view_values)
#     idx += 1
# vis.marker_len = len(markerArray.markers)
# vis.pubmarker.publish(markerArray)

# # frustum marker
# frustum_marker = MarkerArray()    
# idx = 0
# for view in views:
#     val = view_values[view.ID]
#     print idx, val
#     if val > 50:
#         print "Create frustum marker with value", val, len(view.get_keys())
#         vis.create_frustum_marker(frustum_marker, view, view.get_ptu_pose(), view_values)
#     idx += 1
# vis.pubfrustum.publish(frustum_marker)

# cost marker
# cost_marker = MarkerArray()    
# i = 0
# for view1 in views:
#     j = 0
#     for view2 in views:
#         if j <= i:
#             cost = view_costs[view1.ID][view2.ID]
#             print i, j, cost
#             j += 1
#             if cost > 0.0:
#                 print "Create cost marker with value", cost
#                 vis.create_cost_marker(cost_marker, view1, view2, view_costs)
#     i += 1
# vis.pubcost.publish(cost_marker)


with open(INPUT_FILE + "-FILTERED", "w") as outfile:
    json_data = jsonpickle.encode(views)
    outfile.write(json_data)
       
with open(OUTPUT_FILE_VALUES, "w") as outfile:
    json_data = jsonpickle.encode(view_values)
    outfile.write(json_data)

with open(OUTPUT_FILE_VIEW_KEYS, "w") as outfile:
    json_data = jsonpickle.encode(views)
    outfile.write(json_data)

with open(OUTPUT_FILE_COSTS, "w") as outfile:
    json_data = jsonpickle.encode(view_costs)
    outfile.write(json_data)    

rospy.loginfo("Finished view evaluation.")
rospy.spin()
    



