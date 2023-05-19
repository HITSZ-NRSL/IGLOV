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

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Vector3.h>

#include <chrono>
#include <fstream>

using namespace std;

Eigen::Matrix3d R3_world_robot_curr;


Eigen::Matrix4d Get_Twc()
{
  Eigen::Matrix4d t_w_c = Eigen::Matrix4d::Zero();

  std::string camera_frame = "d435i_depth_optical_frame";
  std::string world_frame = "virtual_world";

  tf::TransformListener tf_listener_;
  tf::StampedTransform Camera_World_Tf;

  try
  {
    if (tf_listener_.waitForTransform(world_frame, camera_frame, ros::Time(0), ros::Duration(5)))
    {
      // ROS_WARN_STREAM("time now" << ros::Time::now());
      // ROS_WARN_STREAM("Enter tf");
      tf_listener_.lookupTransform(world_frame, camera_frame, ros::Time(0), Camera_World_Tf);
    }
    // ROS_WARN_STREAM("Frame_id: 0." << cloud_msg->header.frame_id);

  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return t_w_c;
  }

  // Convert tf to matrix
  Eigen::Quaterniond quat;
  quat.w() = Camera_World_Tf.getRotation().getW();
  quat.x() = Camera_World_Tf.getRotation().getX();
  quat.y() = Camera_World_Tf.getRotation().getY();
  quat.z() = Camera_World_Tf.getRotation().getZ();
  t_w_c.block<3,3>(0,0) = quat.toRotationMatrix();
  Eigen::Vector3d translate;
  translate[0] = Camera_World_Tf.getOrigin().getX();
  translate[1] = Camera_World_Tf.getOrigin().getY();
  translate[2] = Camera_World_Tf.getOrigin().getZ();
  t_w_c.block<3, 1>(0, 3) = translate;

  t_w_c.block<1, 4>(3, 0) << 0, 0, 0, 1;

  return t_w_c;
}


Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}

// v_p:point_camera; v_d:direction
Eigen::Vector3d VectorsToEulerAngles(Eigen::Vector3d &v_p, Eigen::Vector3d &v_d)
{
  Eigen::Vector3d v_tem;
  double pitch, yaw;

  if(!((v_p(1) == 0)&&(v_p(0) == 0))){
    v_tem(2) = v_d(2);

    v_tem(0) = v_p(0)*sqrt((1-v_d(2)*v_d(2))/(v_p(1)*v_p(1)+v_p(0)*v_p(0)));

    v_tem(1) = v_p(1)*sqrt((1-v_d(2)*v_d(2))/(v_p(1)*v_p(1)+v_p(0)*v_p(0)));

    double cosValNew = v_p.dot(v_tem) /(v_p.norm()*v_tem.norm()); // cos

    if(v_p(2) > v_d(2)){
      pitch = acos(cosValNew);
    }
    else{
      pitch = -acos(cosValNew);
    }
    

    cosValNew = v_d.dot(v_tem) /(v_d.norm()*v_tem.norm()); // cos

    Eigen::Vector3d cross_vec = v_p.cross(v_d);

    if(cross_vec.z() < 0){
      yaw = -abs(acos(cosValNew));
    }
    else{
      yaw = abs(acos(cosValNew));
    } 
  }
  else{
    double cosValNew = v_p.dot(v_d) /(v_p.norm()*v_d.norm()); // cos

    pitch = acos(cosValNew);

    yaw = 0;
  }

  return Eigen::Vector3d(0, pitch, yaw);
}

Eigen::Matrix4d Twc_initial;
 
geometry_msgs::Vector3 Cal_gimbal_angle(Eigen::Vector3d best_view_point, double pitch_fed, double yaw_fed, Eigen::Matrix4d Twc){

  Eigen::Vector3d euler_angle(0, pitch_fed, yaw_fed);

  Eigen::Vector3d trans_world_camera = Twc.block<3,1>(0,3);

  Eigen::Vector3d view_camera_bestpoint(
      best_view_point.x() - trans_world_camera(0),
      best_view_point.y() - trans_world_camera(1),
      best_view_point.z() - trans_world_camera(2)
  );

  view_camera_bestpoint = view_camera_bestpoint.normalized();

  Eigen::Vector3d identity_e(1,0,0);

  Eigen::Vector3d motion_direction = R3_world_robot_curr * identity_e;
  motion_direction(1) = motion_direction(1);
  motion_direction(2) = - motion_direction(2);

  Eigen::Vector3d euler_angle_opt = VectorsToEulerAngles(view_camera_bestpoint, motion_direction);   

  geometry_msgs::Vector3 euler_angle_MCU;
  euler_angle_MCU.x = euler_angle_opt.x();
  euler_angle_MCU.y = euler_angle_opt.y();    // 
  euler_angle_MCU.z = euler_angle_opt.z();    // 

//                   ^z
//                   |      /x
//                   |     /
//                   |    /
//                   |   /
//                   |  /
//                   | /
//  y<－－－－－－－－－|/

  return euler_angle_MCU;
}