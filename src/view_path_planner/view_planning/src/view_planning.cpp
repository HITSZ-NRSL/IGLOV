/*
 * @file controller.cpp
 * @author Zhihao Wang(19B953031@stu.hit.edu.cn), Haoyao Chen(hychen5@hit.edu.cn), Shiwu Zhang, Yunjiang Lou. 
 * @date 2023-05-20
 * 
 * @copyright Copyright (c) 2023, nROS-Lab, HITsz
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

* You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include "ros/ros.h"
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include <sys/time.h>
#include <mutex>

#include <iostream>
#include <vector>
#include <chrono>
#include <string.h>
#include <map>
#include <unordered_map>
#include <math.h>

#include "infomation_cal.h"
#include "igo_planner.h"

using namespace std;
using namespace octomap;

#define USE_IGLOV_PLANNER 1

// ROS publishers
ros::Publisher vis_pub;
ros::Publisher marker_pub, traject_pub;
ros::Publisher best_view_point_pub;
ros::Publisher candidate_point_pub;
ros::Publisher best_view_pub;
ros::Publisher pub_camera_markers;
ros::Publisher pub_view_direction_markers;

Eigen::Vector3d best_view_point;
Eigen::Matrix3d R3_world_robot_init;

typedef octomap::point3d point3d;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

octomap::OcTree *cur_tree;
octomap_msgs::Octomap msg_octomap;
point3d kinect_orig(0, 0, 0);
double max_range = 7;   // mapping range

std::string camera_frame = "d435i_depth_optical_frame";
std::string world_frame = "world";

vector<geometry_msgs::Point> Visuallization_gwc;
vector<geometry_msgs::Point> Visuallization_bvp;

void Visuallization_Points_History()
{
  visualization_msgs::Marker points, points_opt, points_cur, line_strip, line_list;
  points_cur.header.frame_id = points_opt.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world";
  points_cur.header.stamp = points_opt.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points_cur.ns = points_opt.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points_cur.action = points_opt.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points_cur.pose.orientation.w = points_opt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  visualization_msgs::MarkerArray ma;

  points.id = 0;
  points_cur.id = 1;
  points_opt.id = 2;
  line_strip.id = 3;
  line_list.id = 4;

  points.type = visualization_msgs::Marker::POINTS;
  points_cur.type = visualization_msgs::Marker::POINTS;
  points_opt.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::ARROW;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  points_cur.scale.x = 0.2;
  points_cur.scale.y = 0.2;

  points_opt.scale.x = 0.2;
  points_opt.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.05;
  line_list.scale.x = 0.08;
  line_list.scale.y = 0.4; // arrow head

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  points_cur.color.r = 1.0f;
  points_cur.color.a = 1.0;

  // points_opt.color.b = 1.0;
  points_opt.color.g = 1.0;
  points_opt.color.r = 1.0;
  points_opt.color.a = 1.0;

  // Line strip is blue:连接线
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.0;

  // Line list is red：竖直向上的线
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;

  int i = 0;

  p.x = g_T_w_c_igo(0, 3);
  p.y = g_T_w_c_igo(1, 3);
  p.z = g_T_w_c_igo(2, 3);
  Visuallization_gwc.push_back(p); // save history gwc

  p.x = best_view_point.x();
  p.y = best_view_point.y();
  p.z = best_view_point.z();
  Visuallization_bvp.push_back(p);

  int id = 4;
  for (int i = 0; i < Visuallization_bvp.size(); ++i)
  {
    // ROS_ERROR_STREAM("VISUALIZATION:" << i);

    if (i % 50 == 0)
    {
      line_list.points.clear();
      points_opt.points.clear();
      points_opt.points.push_back(Visuallization_bvp[i]);

      id++;
      points_opt.id = id;
      ma.markers.push_back(points_opt);

      line_strip.points.push_back(Visuallization_bvp[i]);
    }
  }

  marker_pub.publish(line_strip);

  pub_view_direction_markers.publish(ma);
}

void Visuallization_Points(int index, const vector<Eigen::Vector3d> &anchor_points)
{
  visualization_msgs::Marker points, points_opt, points_cur, line_strip, line_list;
  points_cur.header.frame_id = points_opt.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world";
  points_cur.header.stamp = points_opt.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points_cur.ns = points_opt.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points_cur.action = points_opt.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points_cur.pose.orientation.w = points_opt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  points_cur.id = 1;
  points_opt.id = 2;
  line_strip.id = 3;
  line_list.id = 4;

  points.type = visualization_msgs::Marker::POINTS;
  points_cur.type = visualization_msgs::Marker::POINTS;
  points_opt.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::ARROW;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.4;
  points.scale.y = 0.4;

  points_cur.scale.x = 0.2;
  points_cur.scale.y = 0.2;

  points_opt.scale.x = 0.2;
  points_opt.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.05;
  line_list.scale.x = 0.08; // 
  line_list.scale.y = 0.3;  // 

  // Points are green
  points.color.g = 1.0f;
  points.color.r = 1.0f;
  points.color.a = 1.0;

  points_cur.color.r = 1.0f;
  points_cur.color.a = 1.0;

  points_opt.color.g = 1.0f;
  points_opt.color.r = 1.0f;
  points_opt.color.a = 1.0;

  line_strip.color.b = 1.0;
  line_strip.color.a = 0.0;

  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  int i = 0;

  for (auto pt : bp_position_sequence)
  {
    geometry_msgs::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();

    if (i == 0)
    {

      points_cur.points.push_back(p);
      line_strip.points.push_back(p);

      geometry_msgs::Point p_twc;
      p_twc.x = g_T_w_c_igo(0, 3);
      p_twc.y = g_T_w_c_igo(1, 3);
      p_twc.z = g_T_w_c_igo(2, 3);
      line_list.points.push_back(p_twc);

      // The line list needs two points for each line
      line_list.points.push_back(p);
    }
    else if (i > 0)
    {
      points_opt.points.push_back(p);
    }

    i++;
  }

  marker_pub.publish(points);
  marker_pub.publish(points_cur);
  marker_pub.publish(points_opt);
  marker_pub.publish(line_strip);
  marker_pub.publish(line_list);
}

void publishCameraMarker()
{
  /*
   * draw a pyramid as the camera marker
   */

  double marker_scale = 4;
  Eigen::Vector3d color(0, 1, 0);

  const double sqrt2_2 = sqrt(2) / 2;

  constexpr int n_marker = 8;
  const int marker_id_base = 1 * n_marker;

  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;
  // the marker will be displayed in frame_id
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "camera";
  marker.action = visualization_msgs::Marker::ADD;

  // make rectangles as frame
  double r_w = 1.0;
  double z_plane = (r_w / 2.0) * marker_scale;
  marker.pose.position.x = z_plane;
  marker.pose.position.y = (r_w / 2.0) * marker_scale;
  marker.pose.position.z = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = (r_w / 2.0) * marker_scale;
  marker.scale.y = 0.04 * marker_scale;
  marker.scale.z = 0.04 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = sqrt2_2;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = sqrt2_2;
  marker.id = marker_id_base;
  ma.markers.push_back(marker);

  marker.pose.position.y = -(r_w / 2.0) * marker_scale;
  marker.id = marker_id_base + 1;
  ma.markers.push_back(marker);

  marker.scale.x = (r_w)*marker_scale;
  marker.pose.position.x = z_plane;
  marker.pose.position.y = 0;
  marker.pose.position.z = (r_w / 4.0) * marker_scale;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = sqrt2_2;
  marker.pose.orientation.w = sqrt2_2;
  marker.id = marker_id_base + 2;
  ma.markers.push_back(marker);
  marker.pose.position.z = -(r_w / 4.0) * marker_scale;
  marker.id = marker_id_base + 3;
  ma.markers.push_back(marker);

  // make pyramid edges
  marker.scale.x = (3.0 * r_w / 4.0) * marker_scale;
  marker.pose.position.z = (r_w / 8.0) * marker_scale;
  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 4.0) * marker_scale;
  marker.pose.orientation.x = -0.099;
  marker.pose.orientation.y = -0.239;
  marker.pose.orientation.z = 0.370;
  marker.pose.orientation.w = 0.892;

  marker.id = marker_id_base + 4;
  ma.markers.push_back(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;
  marker.pose.orientation.x = 0.099;
  marker.pose.orientation.y = -0.239;
  marker.pose.orientation.z = -0.370;
  marker.pose.orientation.w = 0.892;
  marker.id = marker_id_base + 5;
  ma.markers.push_back(marker);

  marker.pose.position.z = -(r_w / 8.0) * marker_scale;
  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = (r_w / 4.0) * marker_scale;

  marker.pose.orientation.x = 0.099;
  marker.pose.orientation.y = 0.239;
  marker.pose.orientation.z = 0.370;
  marker.pose.orientation.w = 0.892;
  marker.id = marker_id_base + 6;
  ma.markers.push_back(marker);

  marker.pose.position.x = (r_w / 4.0) * marker_scale;
  marker.pose.position.y = -(r_w / 4.0) * marker_scale;

  marker.pose.orientation.x = -0.099;
  marker.pose.orientation.y = 0.239;
  marker.pose.orientation.z = -0.370;
  marker.pose.orientation.w = 0.892;
  marker.id = marker_id_base + 7;
  ma.markers.push_back(marker);

  for (visualization_msgs::Marker &m : ma.markers)
  {
    Eigen::Matrix4d T_om;

    Eigen::Vector3d tran_tag_pose(m.pose.position.x, m.pose.position.y, m.pose.position.z);
    Eigen::Quaterniond quat;
    quat.w() = m.pose.orientation.w;
    quat.x() = m.pose.orientation.x;
    quat.y() = m.pose.orientation.y;
    quat.z() = m.pose.orientation.z;

    T_om.block<3, 3>(0, 0) = quat.toRotationMatrix();
    T_om.block<3, 1>(0, 3) = tran_tag_pose;

    Eigen::Matrix4d Twm = g_T_w_c_igo * T_om;

    m.pose.position.x = Twm(0, 3) + g_T_w_c_igo(0, 3);
    m.pose.position.y = Twm(1, 3) + g_T_w_c_igo(1, 3);
    m.pose.position.z = Twm(2, 3) + g_T_w_c_igo(2, 3);

    Eigen::Quaterniond quaternion2(Twm.block<3, 3>(0, 0));
    m.pose.orientation.w = quaternion2.w();
    m.pose.orientation.x = quaternion2.x();
    m.pose.orientation.y = quaternion2.y();
    m.pose.orientation.z = quaternion2.z();
  }

  pub_camera_markers.publish(ma);
}

void Visuallization_Candidate_Points()
{
  visualization_msgs::Marker points, points_opt, points_cur, line_strip, line_list;
  points_cur.header.frame_id = points_opt.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world";
  points_cur.header.stamp = points_opt.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points_cur.ns = points_opt.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points_cur.action = points_opt.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points_cur.pose.orientation.w = points_opt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;

  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.3;
  points.scale.y = 0.3;

  // Points are green
  points.color.g = 1.0f;
  points.color.b = 1.0f;
  points.color.a = 1.0;

  // Create the vertices for the points and lines
  for (auto pt : Candidate_AP_vec_g)
  {
    geometry_msgs::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();

    points.points.push_back(p);
  }

  candidate_point_pub.publish(points);
}

void Visuallization_Trajectors(int index, const vector<Eigen::Vector3d> &anchor_points)
{
  visualization_msgs::Marker points, points_opt, points_cur, line_strip, line_list;
  points_cur.header.frame_id = points_opt.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/world";
  points_cur.header.stamp = points_opt.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points_cur.ns = points_opt.ns = points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points_cur.action = points_opt.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points_cur.pose.orientation.w = points_opt.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  points_cur.id = 1;
  points_opt.id = 2;
  line_strip.id = 3;
  line_list.id = 4;

  points.type = visualization_msgs::Marker::POINTS;
  points_cur.type = visualization_msgs::Marker::POINTS;
  points_opt.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  points_cur.scale.x = 0.4;
  points_cur.scale.y = 0.4;

  points_opt.scale.x = 0.4;
  points_opt.scale.y = 0.4;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.05;
  line_list.scale.x = 0.1;

  // Points are green
  points.color.b = 1.0f;
  points.color.a = 1.0;

  points_cur.color.b = 1.0f;
  points_cur.color.a = 1.0;

  points_opt.color.b = 1.0f;
  points_opt.color.a = 1.0;

  // Line strip is blue:连接线
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red：竖直向上的线
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  int i = 0;
  // Create the vertices for the points and lines
  for (auto pt : anchor_points)
  {
    geometry_msgs::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();

    if (i == index)
    {
      points_cur.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      line_list.points.push_back(p);
    }
    else
    {
      if (Anchor_points_fixed[i] == false)
      {
        points_opt.points.push_back(p);
      }
      else if (Anchor_points_fixed[i] == true)
      {
        points.points.push_back(p);
      }

      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);

      line_list.points.push_back(p);
    }

    i++;
  }

  traject_pub.publish(points);
  traject_pub.publish(points_cur);
  traject_pub.publish(points_opt);
  traject_pub.publish(line_strip);
  traject_pub.publish(line_list);
}

const int N = 999;
double random_noise()
{
  return (rand() % (N + 1) / (float)(N + 1)) * 2 - 1;
}

double random_noise(double depth_)
{
  return ((rand() % (N + 1) / (float)(N + 1)) * 2 - 1) * depth_;
}

void insertFeatureCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
  // ROS_WARN_STREAM("insertFeatureCloudCallback");
  tf::StampedTransform sensorToWorldTf;
  tf::TransformListener tf_listener;

  tf::TransformListener tf_listener_;
  tf::StampedTransform Camera_World_Tf;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  printf("insertFeatureCloudCallback second:%ld \n", tv.tv_sec);      // s
  printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); // ms

  PointCloud *cloud(new PointCloud);
  PointCloud *cloud_local(new PointCloud);

  pcl::fromROSMsg(*cloud_msg, *cloud);

  octomap::Pointcloud hits;

  density_map.clear();

  Eigen::Vector4d pwi(0, 0, 0, 1);
  Eigen::Vector4d pci(0, 0, 0, 1);

  for (auto p : cloud->points)
  {
    pwi.x() = p.x;
    pwi.y() = p.y;
    pwi.z() = p.z;

    // add noise for verification, this part can be igonore
    pci = g_T_w_c_igo.inverse() * pwi;
    double depth_ = pci.x();
    pci.x() += random_noise() * 0.001425 * depth_ * depth_;
    pci.y() += random_noise() * 0.01 * depth_;
    pci.z() += random_noise() * 0.01 * depth_;
    pwi = g_T_w_c_igo * pci;

    p.x = pwi.x();
    p.y = pwi.y();
    p.z = pwi.z();

    if (std::abs(p.x) < 0.05 && std::abs(p.y) < 0.05)   // boundary condition
    {
      continue;
    }
    if ((std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)))   // boundary condition
    {
      continue;
    }
    else
    {

      int index_x, index_y, index_z;
      if (p.x > 0)
      {
        index_x = floor((p.x + Octomap_Resolution_) / Octomap_Resolution_);
      }
      else
      {
        index_x = floor(p.x / Octomap_Resolution_);
      }

      if (p.y > 0)
      {
        index_y = floor((p.y + Octomap_Resolution_) / Octomap_Resolution_);
      }
      else
      {
        index_y = floor(p.y / Octomap_Resolution_);
      }

      if (p.z > 0)
      {
        index_z = floor((p.z + Octomap_Resolution_) / Octomap_Resolution_);
      }
      else
      {
        index_z = floor(p.z / Octomap_Resolution_);
      }

      auto search = density_map.find(Node(index_x, index_y, index_z));
      if (search != density_map.end())
      {
        search->second += 1;
        if (search->second >= 1000)   // feature is enough
        {
          search->second = 1000;
        }
      }
      else
      {
        density_map[Node(index_x, index_y, index_z)] = 1;
        hits.push_back(p.x, p.y, p.z);
      }
    }
  }
  cout << "density_map.size(): " << density_map.size() << endl;

  if (density_map.size() <= 0)
  {
    cout << "map not initialized..." << endl;
    return;
  }

  kinect_orig.x() = g_T_w_c_igo(0, 3);
  kinect_orig.y() = g_T_w_c_igo(1, 3);
  kinect_orig.z() = g_T_w_c_igo(2, 3);

  cur_tree->insertPointCloud(hits, kinect_orig, max_range);

#if USE_IGLOV_PLANNER
  igo_view_planner(cur_tree);

  best_view_point = run_IGO_ViewPlanner();   // view planning

  Visuallization_Points_History();
#endif

  // print and save
#if 0
    CalAllCell_Infos_Save(cur_tree);
#endif

  ros::Time current_time;
  current_time = ros::Time::now();
  geometry_msgs::PointStamped best_viewPoint_stamped;
  best_viewPoint_stamped.header.stamp = current_time;
  best_viewPoint_stamped.header.frame_id = "world";
  best_viewPoint_stamped.point.x = best_view_point.x();
  best_viewPoint_stamped.point.y = best_view_point.y();
  best_viewPoint_stamped.point.z = best_view_point.z();

  best_view_point_pub.publish(best_viewPoint_stamped);

  delete cloud;
  delete cloud_local;
}

void orbpose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  struct timeval tv;
  gettimeofday(&tv, NULL);
  printf("orbpose_callback second:%ld \n", tv.tv_sec);                // 秒
  printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); // 毫秒

  Eigen::Quaterniond quat;
  quat.w() = msg->pose.orientation.w;
  quat.x() = msg->pose.orientation.x;
  quat.y() = msg->pose.orientation.y;
  quat.z() = msg->pose.orientation.z;
  g_T_w_c_igo.block<3, 3>(0, 0) = quat.toRotationMatrix();
  Eigen::Vector3d translate;
  translate[0] = msg->pose.position.x;
  translate[1] = msg->pose.position.y;
  translate[2] = msg->pose.position.z + 0.404; // orb发布的是world到baselink的坐标，需要添加baselink到camera的高度
  g_T_w_c_igo.block<3, 1>(0, 3) = translate;

  ROS_INFO_STREAM("Current posiiton: " << translate.transpose());

  // cout << "ORBSLAM pose callback g_T_w_c_igo: \n" << g_T_w_c_igo << endl;
}

// body_imu for getting direction of robot base.
static int count_imu = 0;
void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  printf("second:%ld \n", tv.tv_sec);                                 // s
  printf("millisecond:%ld \n", tv.tv_sec * 1000 + tv.tv_usec / 1000); // ms

  Eigen::Quaterniond q2(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  if (count_imu <= 10)
  {
    R3_world_robot_init = q2.matrix();
    count_imu++;
    cout << "count_imu: " << count_imu << endl;
    cout << "R3_world_robot_init: " << R3_world_robot_init << endl;
  }
  else
  {
    R3_world_robot_curr = q2.matrix();
    // cout << "R3_world_robot_curr: " << R3_world_robot_curr << endl;

    Body_dir = R3_world_robot_init.inverse() * R3_world_robot_curr * Body_dir_init;
    Body_dir.z() = 0;
    Body_dir = Body_dir.normalized();

    // filter the base direction
    Dir_queue.pop_front();
    Dir_queue.push_back(Body_dir);

    Eigen::Vector3d Body_dir_sum(0, 0, 0);
    for (auto it = Dir_queue.begin(); it != Dir_queue.end(); it++)
    {
      Body_dir_sum.x() += (*it).x();
      Body_dir_sum.y() += (*it).y();
      Body_dir_sum.z() += (*it).z();
    }

    Body_dir_avg = Body_dir_sum / 5;
    cout << "Body_dir_avg: " << Body_dir_avg.transpose() << endl;
  }
}

int Anchor_index_old = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "view_planner");
  ros::NodeHandle n;

  // SUBSCRIBER
  ros::Subscriber orbpose_sub = n.subscribe<geometry_msgs::PoseStamped>("/orb_slam2_rgbd/pose", 1, orbpose_callback);
  ros::Subscriber sub_view_robot_camera = n.subscribe<sensor_msgs::Imu>("/body_imu", 1, body_imu_callback);
  ros::Subscriber sub_orbslam_mappoints = n.subscribe<sensor_msgs::PointCloud2>("orb_slam2_rgbd/map_points", 1, &insertFeatureCloudCallback);

  // PUBLISHER
  best_view_point_pub = n.advertise<geometry_msgs::PointStamped>("best_view_point", 10, true);

  // PUBLISHER for VISUALIZATION
  ros::Publisher Octomap_pub = n.advertise<octomap_msgs::Octomap>("octomap_feature3d", 1);
  candidate_point_pub = n.advertise<visualization_msgs::Marker>("visualization_candidate_points", 10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  traject_pub = n.advertise<visualization_msgs::Marker>("visualization_traject", 10);
  pub_camera_markers = n.advertise<visualization_msgs::MarkerArray>("camera_markers", 10);
  pub_view_direction_markers = n.advertise<visualization_msgs::MarkerArray>("view_direction_markers", 10);

  octomap::OcTree new_tree(Octomap_Resolution_);
  cur_tree = &new_tree;

  Init_Anchor_points();
  Init_Sample_points();

  Rot_90 << -0.0, -1.0, 0.0,
      1.0, -0.0, 0.0,
      0.0, 0.0, 1.0;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {

    // pub octomap for visualization
    octomap_msgs::fullMapToMsg(*cur_tree, msg_octomap);
    msg_octomap.header.frame_id = world_frame;
    msg_octomap.header.stamp = ros::Time::now();
    Octomap_pub.publish(msg_octomap);

    if (Anchor_index > Anchor_index_old)
    {
      Anchor_index_old = Anchor_index;
      Visuallization_Trajectors(Anchor_index - 1, Anchor_points); // Anchor Points trajectory

      Visuallization_Points(Anchor_index - 1, Sample_points); // Best View points trajectory

      Visuallization_Candidate_Points();

      publishCameraMarker();
    }


    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
