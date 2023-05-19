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

#ifndef INFO_CAL_H
#define INFO_CAL_H

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
#include <unordered_map>

#include <chrono>

using namespace std;

typedef Eigen::Matrix<double, 3, 6> Matrix36;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 3, 4> Matrix34;
typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 6, 6> Matrix66;

struct Node
{
  int x;
  int y;
  int z;

  Node(int a, int b, int c) : x(a), y(b), z(c) {}

  bool operator==(const Node &p) const
  {
    return x == p.x && y == p.y && z == p.z;
  }
};

// Way-1: specialized hash function for unordered_map keys
struct NodeHash
{
  std::size_t operator()(const Node &node) const
  {
    return (node.x * 1483 + node.y * 2791 + node.z * 4363);
  }
};

extern Eigen::Matrix4d g_T_w_c_igo;
extern double Octomap_Resolution_;

extern unordered_map<Node, int, NodeHash> density_map;

extern octomap::OcTree *maptree_igo_;


void save_time_consuming(const std::string file_name, float time);


double Get_Max_z(Eigen::Vector3d point);    //

int Get_Zaxis_Density(Eigen::Vector3d point);   //

int Get_Neighbor_Density(Eigen::Vector3d point);

float Get_Neighbor_Distribution(Eigen::Vector3d point);

float Get_Neighbor_FisherInfo(Eigen::Vector3d point);

float Get_Neighbor_Info(Eigen::Vector3d point);


void CalAllCell_Infos_Save(octomap::OcTree *map_tree_);

#endif /* INFO_CAL_H */