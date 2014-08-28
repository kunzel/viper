#include <stdlib.h>  
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "octomap_msgs/GetOctomap.h"
using octomap_msgs::GetOctomap;
#include <octomap_msgs/conversions.h>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <viper/ViewValue.h>

#include "frustum.cpp"

using namespace std;
using namespace octomap;

#define ANGLE_MAX_DIFF (M_PI / 8)  
double frustum_near = 0.8;
double frustum_far = 2.5;
double frustum_angle = 40.5;
double frustum_ratio = 1.333;

OcTree* extract_supporting_planes(OcTree* tree)
{
  OcTree* sp_tree = new OcTree(tree->getResolution());  

  int free = 0;
  int occupied = 0;
  int supported = 0;
  
  ROS_INFO("Extracting supporting planes from octomap");
  
  for(OcTree::leaf_iterator it = tree->begin_leafs(),
        end=tree->end_leafs(); it!= end; ++it)
    {
      if (tree->isNodeOccupied(*it))
        {
          occupied++;
          std::vector<point3d> normals;
          
          point3d p3d = it.getCoordinate();

          bool got_normals = tree->getNormals(p3d ,normals, true); 
          std::vector<point3d>::iterator normal_iter;
          
          point3d avg_normal (0.0, 0.0, 0.0);
          for(std::vector<point3d>::iterator normal_iter = normals.begin(), 
                end = normals.end(); normal_iter!= end; ++normal_iter)
            {
              avg_normal+= (*normal_iter);
            }
          if (normals.size() > 0) 
            {
              supported++;
              // cout << "#Normals: " << normals.size() << endl;

              avg_normal/= normals.size();       
              
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);

              point3d coord = it.getCoordinate();

              if ( angle < ANGLE_MAX_DIFF)
                {
                  sp_tree->updateNode(coord,true);
                } 
            }  
        } 
      else 
        {
          free++;
        }
    }
  ROS_INFO("Extracted map size: %i (%i free, and %i occupied leaf nodes were discarded)", supported, free, occupied - supported);
  return sp_tree;
}

OcTree* retrieve_octree()
{
  ros::NodeHandle n;
  std::string servname = "octomap_binary";
  ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
  GetOctomap::Request req;
  GetOctomap::Response resp;
  while(n.ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }
 
  OcTree* octree = octomap_msgs::binaryMsgToMap(resp.map);

  if (octree){
    ROS_INFO("Map received (%zu nodes, %f m res)", octree->size(), octree->getResolution());
    return extract_supporting_planes(octree);
  }
  return NULL;
}

OcTree* input_tree;


int compute_value(Frustum frustum)
{  
  int value = 0;
  int free = 0;
  int occupied = 0;

  for(OcTree::leaf_iterator it = input_tree->begin_leafs(),
        end=input_tree->end_leafs(); it!= end; ++it)
    {
      if (input_tree->isNodeOccupied(*it))
        {
          int size = (int) (it.getSize() / input_tree->getResolution());
          //ROS_INFO("Size %f", size); 
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          occupied++;
          Vec3 point(x, y, z);
          if (frustum.is_inside(point))
            {
              //ROS_INFO("Node inside frustum");
              value += 1 * size;
            }
        }
      else 
        {
          free++;
        }
    }
  //std::cerr<<"occupied "<< occupied<< std::endl;
  //std::cerr<<"free "<< free<< std::endl;
  return value;
}

Frustum generate_local_frustum()
{
  Frustum frustum_temp;

  Vec3 p(0.0, 0.0, 0.0);//camera_height); // Camera position.
  Vec3 l(1.0, 0.0, 0.0);//camera_height); // Look at vector.
  Vec3 u(0.0, 0.0, 1.0); // Right vector.
  frustum_temp.setCamInternals(frustum_angle, frustum_ratio, frustum_near, frustum_far);
  frustum_temp.setCamDef(p, l, u);
  return frustum_temp;
}


Frustum generate_frustum(geometry_msgs::Pose pose)
{
  
  Vec3 points[8];
  Frustum frustum, frustum_temp;

  Vec3 p(0.0, 0.0, 0.0);//camera_height); // Camera position.
  Vec3 l(1.0, 0.0, 0.0);//camera_height); // Look at vector.
  Vec3 u(0.0, 0.0, 1.0); // Right vector.
  frustum_temp.setCamInternals(frustum_angle, frustum_ratio, frustum_near, frustum_far);
  frustum_temp.setCamDef(p, l, u);
  points[0].x = frustum_temp.ntl.x;
  points[0].y = frustum_temp.ntl.y;
  points[0].z = frustum_temp.ntl.z;
  points[1].x = frustum_temp.ntr.x;
  points[1].y = frustum_temp.ntr.y;
  points[1].z = frustum_temp.ntr.z;
  points[2].x = frustum_temp.nbl.x;
  points[2].y = frustum_temp.nbl.y;
  points[2].z = frustum_temp.nbl.z;
  points[3].x = frustum_temp.nbr.x;
  points[3].y = frustum_temp.nbr.y;
  points[3].z = frustum_temp.nbr.z;

  points[4].x = frustum_temp.ftl.x;
  points[4].y = frustum_temp.ftl.y;
  points[4].z = frustum_temp.ftl.z;
  points[5].x = frustum_temp.ftr.x;
  points[5].y = frustum_temp.ftr.y;
  points[5].z = frustum_temp.ftr.z;
  points[6].x = frustum_temp.fbl.x;
  points[6].y = frustum_temp.fbl.y;
  points[6].z = frustum_temp.fbl.z;
  points[7].x = frustum_temp.fbr.x;
  points[7].y = frustum_temp.fbr.y;
  points[7].z = frustum_temp.fbr.z;  

	tf::Transform tf_pose;
	Vec3 points_new[8];

	tf::poseMsgToTF(pose, tf_pose);
	for (int i = 0; i < 8; i++)
    {
      
      tf::Vector3 old_pos(points[i].x, points[i].y, points[i].z);

      tf::Vector3 new_pos;
      new_pos = tf_pose * old_pos;
      points_new[i].x = new_pos.getX();
      points_new[i].y = new_pos.getY();
      points_new[i].z = new_pos.getZ();
    }

	
	// When not considered the frustum parameters and hardcoded frustum values:

	frustum.ntl.x = points_new[0].x;
	frustum.ntl.y = points_new[0].y;
	frustum.ntl.z = points_new[0].z;
	frustum.ntr.x = points_new[1].x;
	frustum.ntr.y = points_new[1].y;
	frustum.ntr.z = points_new[1].z;
	frustum.nbl.x = points_new[2].x;
	frustum.nbl.y = points_new[2].y;
	frustum.nbl.z = points_new[2].z;
	frustum.nbr.x = points_new[3].x;
	frustum.nbr.y = points_new[3].y;
	frustum.nbr.z = points_new[3].z;

	frustum.ftl.x = points_new[4].x;
	frustum.ftl.y = points_new[4].y;
	frustum.ftl.z = points_new[4].z;
	frustum.ftr.x = points_new[5].x;
	frustum.ftr.y = points_new[5].y;
	frustum.ftr.z = points_new[5].z;
	frustum.fbl.x = points_new[6].x;
	frustum.fbl.y = points_new[6].y;
	frustum.fbl.z = points_new[6].z;
	frustum.fbr.x = points_new[7].x;
	frustum.fbr.y = points_new[7].y;
	frustum.fbr.z = points_new[7].z;
	
	frustum.setCamDef();
  return frustum;
}

geometry_msgs::Point vec_to_point(Vec3 vec)
{
  geometry_msgs::Point point;
  point.x = vec.x;
  point.y = vec.y;
  point.z = vec.z;
  return point;
}

std::vector<geometry_msgs::Point> get_points(Frustum f)
{
  std::vector<geometry_msgs::Point> points;
  
  points.push_back(vec_to_point(f.ntl));
  points.push_back(vec_to_point(f.ntr));
  points.push_back(vec_to_point(f.nbl));
  points.push_back(vec_to_point(f.nbr));
  points.push_back(vec_to_point(f.ftl));
  points.push_back(vec_to_point(f.ftr));
  points.push_back(vec_to_point(f.fbl));
  points.push_back(vec_to_point(f.fbr));

  return points;
}


// Callback function for the service 'view_eval'
bool view_eval(viper::ViewValue::Request  &req,
               viper::ViewValue::Response &res)
{
  
  ROS_INFO("Received service request.");
  geometry_msgs::Pose camera_pose = req.pose; 

	// Generate frustum.
  Frustum f = generate_frustum(camera_pose);
	
  int value = compute_value(f);
  
  res.value = value;
  res.frustum = get_points(generate_local_frustum());

  ROS_INFO("Finished service request.");
  return true;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "evaluate_view");
  ros::NodeHandle node; 

  //node.getParam("frustum_near", frustum_near);
  //node.getParam("frustum_far", frustum_far);
  //node.getParam("frustum_angle", frustum_angle);

  input_tree = retrieve_octree();  
  
  ros::ServiceServer view_eval_service = node.advertiseService("view_eval", view_eval);


  ROS_INFO("Started view evaluation service");
  ros::spin();
  ROS_INFO("Stopped view evaluation service");
  return 0;
}










