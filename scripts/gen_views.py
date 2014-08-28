#!/usr/bin/env python
import rospy
import jsonpickle
from viper.core.planner import ViewPlanner
from geometry_msgs.msg import PoseArray 

robot_poses_pub = rospy.Publisher('robot_poses', PoseArray)
ptu_poses_pub = rospy.Publisher('ptu_poses', PoseArray)
rospy.init_node('view_generation')

import viper.robots.scitos
robot = viper.robots.scitos.ScitosRobot()

NUM_OF_VIEWS = rospy.get_param('~num_of_views', 100)
FILENAME     = rospy.get_param('~output_file', 'views.json')

planner = ViewPlanner(robot)

rospy.loginfo('Generate views.')
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
    

