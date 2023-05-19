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

#ifndef IGO_PLANNER_H
#define IGO_PLANNER_H

/*
Information Gradient Local View Planner-IGO view planner
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <vector>
#include <iostream>
#include <octomap/octomap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
// #include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <chrono>

#include "infomation_cal.h"
#include <numeric>

#include <unordered_map>
#include <queue>
#include <list>
#include <mutex>

using namespace std;
using namespace Eigen;

#define TIME_CONSUMING_COUNT 0

// Preset Trajectory
extern vector<Eigen::Vector3d> Anchor_points;

extern Eigen::Matrix3d R3_world_robot_curr;

extern Eigen::Vector3d Body_dir;
extern Eigen::Vector3d Body_dir_init;
extern std::list<Eigen::Vector3d> Dir_queue;
extern Eigen::Vector3d Body_dir_avg;
extern Eigen::Vector3d Body_dir_avg_last;

extern Eigen::Matrix3d Rot_90;
extern Eigen::Vector3d Dir_search;

// Eigen::Vector3d Init_dir(1,0,0);
// Path
extern vector<Eigen::Vector3d> Sample_points;
extern vector<bool> Anchor_points_fixed;
extern std::vector<Eigen::Vector3d> bp_position_sequence; // mpc优化点序列

extern int Anchor_index;

extern std::vector<Eigen::Vector3d> Candidate_AP_vec_g;

extern double Search_Angle; // 搜索角度范围


// 初始化参考轨迹
void Init_Anchor_points();

// 初始化锚点
void Init_Sample_points();

void igo_view_planner(octomap::OcTree *map_tree);

Eigen::Vector3d run_IGO_ViewPlanner();

#endif /* IGO_PLANNER_H */
