#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner
from geometry_msgs.msg import PoseArray 

from viper.srv import GetKeys, GetKeysRequest


robot_poses_pub = rospy.Publisher('robot_poses', PoseArray)
ptu_poses_pub = rospy.Publisher('ptu_poses', PoseArray)
rospy.init_node('view_generation')

import viper.robots.scitos_coverage
robot = viper.robots.scitos_coverage.ScitosRobot()

NUM_OF_VIEWS = rospy.get_param('~num_of_views', 100)
FILENAME     = rospy.get_param('~output_file', 'views.json')
COVERAGE     = rospy.get_param('~coverage', False)


from soma_roi_manager.soma_roi import SOMAROIQuery
soma_map  = "g4s"
soma_conf = "ijcai"
roi_id   = '1'
soma = SOMAROIQuery(soma_map, soma_conf)
poly = soma.get_polygon(roi_id)

# set ROI param 
polygon = []
for p in poly.points:
    polygon.append([p.x, p.y])
rospy.set_param('roi', polygon)
rospy.loginfo(polygon)


planner = ViewPlanner(robot)

rospy.loginfo('Generate views.')
if COVERAGE == True:
    MIN_COVERAGE = rospy.get_param('~min_coverage', 0.95)
    from octomap_msgs.msg import Octomap
    from octomap_msgs.srv import GetOctomap, GetOctomapRequest

    octomap = Octomap()
    # #octomap_service_name = '/Semantic_map_publisher_node/SemanticMapPublisher/ObservationOctomapService'
    # rospy.loginfo("Waiting for octomap service")
    # octomap_service_name = '/octomap_full'
    # rospy.wait_for_service(octomap_service_name)
    # rospy.loginfo("Done")
    # try:
    #     octomap_service = rospy.ServiceProxy(octomap_service_name, GetOctomap)
    #     req = GetOctomapRequest()
    #     rospy.loginfo("Requesting octomap from semantic map service")
    #     res = octomap_service(req)
    #     octomap = res.map
    #     rospy.loginfo("Received octomap: size:%s resolution:%s", len(octomap.data), octomap.resolution)
    
    # except rospy.ServiceException, e:
    #     rospy.logerr("Service call failed: %s"%e)

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
        print "OCTOMAP KEYS", octomap_keys
    
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)
    
    views = planner.sample_views_coverage(NUM_OF_VIEWS, MIN_COVERAGE, octomap, octomap_keys)
else:
    views = planner.sample_views(NUM_OF_VIEWS)
rospy.loginfo('Generate views. Done.')

robot_poses  = PoseArray()
robot_poses.header.frame_id = '/map'
robot_poses.poses = []
for v in views:
    robot_poses.poses.append(v.get_robot_pose())
print len(robot_poses.poses)
robot_poses_pub.publish(robot_poses)

ptu_poses  = PoseArray()
ptu_poses.header.frame_id = '/map'
ptu_poses.poses = []
for v in views:
    ptu_poses.poses.append(v.get_ptu_pose())
print len(ptu_poses.poses)
ptu_poses_pub.publish(ptu_poses)

with open(FILENAME, "w") as outfile:
    json_data = jsonpickle.encode(views)
    outfile.write(json_data)

rospy.spin()
    

