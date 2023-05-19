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

#include <string.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <math.h>

#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include "Eigen/Core"


#include "controller.hpp"
#include "timercpp.h"

using namespace std;

//ROS publishers
ros::Publisher best_view_pub;

Eigen::Matrix4d G_T_wc = Eigen::Matrix4d::Identity();
Eigen::Vector3d best_view_point;

double pitch_fed = 0, yaw_fed = 0;
geometry_msgs::Vector3 euler_angle2MCU;

std::string camera_frame = "d435i_depth_optical_frame";
std::string world_frame = "virtual_world";


void best_view_point_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  best_view_point.x() = msg->point.x;
  best_view_point.y() = msg->point.y;
  best_view_point.z() = msg->point.z;
}

void orbpose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Eigen::Quaterniond quat;
  quat.w() = msg->pose.orientation.w;
  quat.x() = msg->pose.orientation.x;
  quat.y() = msg->pose.orientation.y;
  quat.z() = msg->pose.orientation.z;
  G_T_wc.block<3,3>(0,0) = quat.toRotationMatrix();
  Eigen::Vector3d translate;
  translate[0] = msg->pose.position.x;
  translate[1] = msg->pose.position.y;
  translate[2] = msg->pose.position.z + 0.404;    
  G_T_wc.block<3, 1>(0, 3) = translate;
}

void jointstates_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  pitch_fed = msg->position[6];
  yaw_fed = msg->position[7];
}

void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg){
  Eigen::Quaterniond q2(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

  R3_world_robot_curr = q2.matrix();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal_planner");
  ros::NodeHandle n;

  ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>("/amphi_robot/joint_states", 1, jointstates_callback);

  ros::Subscriber orbpose_sub = n.subscribe<geometry_msgs::PoseStamped>("/orb_slam2_rgbd/pose", 1, orbpose_callback);

  ros::Subscriber bestviewpoint_sub = n.subscribe<geometry_msgs::PointStamped>("/best_view_point", 1, best_view_point_callback);

  // body_imu for getting direction of robot base
  ros::Subscriber sub_view_robot_camera = n.subscribe<sensor_msgs::Imu>("/body_imu", 1, body_imu_callback);


  best_view_pub = n.advertise<geometry_msgs::Vector3>("/best_view_robot", 1, true);

  // Timer
  Timer t;
  t.setInterval([&]() {
    {
      if(!(std::abs(best_view_point.x()) <= 0.05 && std::abs(best_view_point.y()) <= 0.05 && std::abs(best_view_point.z()) <= 0.05)){
        // controller for gimbal
        euler_angle2MCU = Cal_gimbal_angle(best_view_point, pitch_fed, yaw_fed, G_T_wc);    

        best_view_pub.publish(euler_angle2MCU);
      }
    }

  }, 100); 

  ros::Rate loop_rate(50);

  while (ros::ok())
  {  
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}
