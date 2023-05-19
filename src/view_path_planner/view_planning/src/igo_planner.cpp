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

#include "igo_planner.h"

vector<Eigen::Vector3d> Sample_points;
vector<Eigen::Vector3d> Anchor_points;
int Anchor_index = 0;

Eigen::Matrix3d Rot_90;
Eigen::Vector3d Dir_search;
Eigen::Vector3d Body_dir_avg(1, 0, 0);
Eigen::Vector3d Body_dir_init(1, 0, 0);
std::list<Eigen::Vector3d> Dir_queue(5, Body_dir);
Eigen::Vector3d Body_dir(1, 0, 0);
Eigen::Vector3d Body_dir_avg_last(1, 0, 0);

Eigen::Matrix3d R3_world_robot_curr;

vector<bool> Anchor_points_fixed(1000, false);
std::vector<Eigen::Vector3d> bp_position_sequence; // mpc优化点序列
std::vector<Eigen::Vector3d> Candidate_AP_vec_g;
std::vector<int> bp_index_sequence;

// local
const int Anchor_size = 10; // 优化的锚点的个数
float Anchor_step_size = 0.4;
int Threshold_AP_Density = 3;      // 锚点密度阈值，必须大于阈值才能作为锚点
double Trajector_Grow_Start = 1.8; //
Eigen::Vector3d Anchor_init(Trajector_Grow_Start, -0.0, 0.4);
double Horizon_search_range = 10; // 8×Octomap_Resolution_的距离

Eigen::Vector3d current_position_;
Eigen::Vector3d current_position_last;

Eigen::Vector3d best_view_point_;

std::vector<float> AP_info;
double alpha_distance = 10;
double alpha_density = 1;
double Search_Angle = (M_PI * 6) / 3; // 搜索角度范围

int poly_order = 6; // 拟合阶数
float polyfit_result[Anchor_size][7];

double wInfo_gradient_ = 0.4;
double wSmoothness_ = 0.06;

float Circle_Index[Anchor_size] = {0};

int max_iterator_ = 10;

int N_ap = 5; // N_ap: max sensor range
int Predict_step = 5;

double second_sum = 0;
int second_count = 0;

void Init_Anchor_points()
{
  Eigen::Vector3d p_tm = Anchor_init;
  for (int i = 0; i < Anchor_size; ++i)
  {
    p_tm.x() = Anchor_init.x() + i * Anchor_step_size;
    Anchor_points.push_back(p_tm);
  }
}

void Init_Sample_points()
{
  Eigen::Vector3d p_tm = Anchor_init;
  for (int i = 0; i < Anchor_size; ++i)
  {
    p_tm.x() = Anchor_init.x() + i * Anchor_step_size;
    Sample_points.push_back(p_tm);
  }
}

void igo_view_planner(octomap::OcTree *map_tree)
{
  maptree_igo_ = map_tree;

  std::cout << "Update Feature_Octomap." << std::endl;
}

void Generate_TP_()
{
  Anchor_points.clear();
  // cout << "Anchor_index: " << anchor_index_ << endl;
  for (int i = 0; i < Anchor_size; ++i)
  {
    Anchor_points.push_back(current_position_ + Body_dir_avg * (Trajector_Grow_Start + i * Anchor_step_size));
  }
}


/**
 * @brief Fit polynomial using Least Square Method.
 *
 * @param X X-axis coordinate vector of sample data.
 * @param Y Y-axis coordinate vector of sample data.
 * @param orders Fitting order which should be larger than zero.
 * @return Eigen::VectorXf Coefficients vector of fitted polynomial.
 */
Eigen::VectorXf FitterLeastSquareMethod(vector<float> &X, vector<float> &Y, uint8_t orders)
{
  // abnormal input verification
  if (X.size() < 2 || Y.size() < 2 || X.size() != Y.size() || orders < 1)
    exit(EXIT_FAILURE);

  // map sample data from STL vector to eigen vector
  Eigen::Map<Eigen::VectorXf> sampleX(X.data(), X.size());
  Eigen::Map<Eigen::VectorXf> sampleY(Y.data(), Y.size());

  Eigen::MatrixXf mtxVandermonde(X.size(), orders + 1); // Vandermonde matrix of X-axis coordinate vector of sample data
  Eigen::VectorXf colVandermonde = sampleX;             // Vandermonde column

  // construct Vandermonde matrix column by column
  for (size_t i = 0; i < orders + 1; ++i)
  {
    if (0 == i)
    {
      mtxVandermonde.col(0) = Eigen::VectorXf::Constant(X.size(), 1, 1);
      continue;
    }
    if (1 == i)
    {
      mtxVandermonde.col(1) = colVandermonde;
      continue;
    }
    colVandermonde = colVandermonde.array() * sampleX.array();
    mtxVandermonde.col(i) = colVandermonde;
  }

  // calculate coefficients vector of fitted polynomial
  Eigen::VectorXf result = (mtxVandermonde.transpose() * mtxVandermonde).inverse() * (mtxVandermonde.transpose()) * sampleY;

  return result;
}

// My modification for division. There should be integer division
template <class T, class Q>
std::vector<T> operator/(const std::vector<T> &A, const Q c)
{

  std::vector<T> R(A.size());
  std::transform(A.begin(), A.end(), R.begin(),
                 std::bind1st(std::divides<T>(), c));
  return R;
}

void Circle_Sampling(const Eigen::Vector3d &AP_ref, vector<float> & Candidate_AP_density, vector<float> & Candidate_AP_index){
  float density_local = 0;

  double ring_radius = (AP_ref - current_position_).norm();
  // cout << "ring_radius: " << ring_radius << endl;
  double x, y, th;

  Eigen::Vector3d Init_Vec(1, 0, 0);
  double th_bias = acos(Init_Vec.dot(Body_dir_avg) / (Body_dir_avg.norm()));
  // cout << "th_bias: " << th_bias << endl;

  for (int i = (-1) * Horizon_search_range; i <= Horizon_search_range; ++i)
  {
    th = i * (Search_Angle / (2 * Horizon_search_range)) - th_bias;
    // cout << "th: " << th << endl;
    x = ring_radius * cos(th);
    // cout << "x: " << x << endl;
    y = ring_radius * sin(th);
    // cout << "y: " << y << endl;

    Eigen::Vector3d Decrease_Vec(x, y, 0);
    // std::cout << "Decrease_Vec position: " << Decrease_Vec.transpose() << std::endl;

    // Eigen::Vector3d Candidate_AP(AP_ref.x(), AP_ref.y() + i*Octomap_Resolution_, AP_ref.z());
    Eigen::Vector3d Candidate_AP(current_position_ + Decrease_Vec);

    Candidate_AP_vec_g.push_back(Candidate_AP);

    density_local = Get_Neighbor_Info(Candidate_AP);

    alpha_distance = density_local / Horizon_search_range;
    density_local = density_local * alpha_density - abs(i) * alpha_distance;

    // std::cout << "Candidate_AP position: " << Candidate_AP.transpose() << std::endl;
    // std::cout << "Candidate_AP density: " << density_local << std::endl;

    // if(density_local > Threshold_AP_Density)
    if (density_local > 3)
    {
      Candidate_AP_density.push_back(density_local);
      Candidate_AP_index.push_back(i);
    }
    else
    {
      // std::cout << "Candidate_AP position: " << Candidate_AP.x() << " " << Candidate_AP.y() << " " << Candidate_AP.z() << " "  << std::endl;
      // std::cout << "Candidate_AP Index: " << i << std::endl;
      // std::cout << "Candidate_AP density: " << 0 << std::endl;

      Candidate_AP_density.push_back(0);
      Candidate_AP_index.push_back(i);
    }

    density_local = 0;
  }
}

void Rectangle_Sampling(const Eigen::Vector3d &AP_ref, vector<float> & Candidate_AP_density, vector<float> & Candidate_AP_index){
  float density_local = 0;

  for (int i = (-1) * Horizon_search_range; i <= Horizon_search_range; i += 1)
  {

    // Eigen::Vector3d Candidate_AP(AP_ref.x(), AP_ref.y() + i*Octomap_Resolution_, AP_ref.z());
    Eigen::Vector3d Candidate_AP(AP_ref + Dir_search * i * Octomap_Resolution_);

    Candidate_AP_vec_g.push_back(Candidate_AP);

    density_local = Get_Neighbor_Info(Candidate_AP);

    alpha_distance = density_local / Horizon_search_range;
    double density_local_ = density_local * alpha_density - abs(i) * alpha_distance;

    if (density_local > 3)
    {
      // std::cout << "Candidate_AP position: " << Candidate_AP.x() << " " << Candidate_AP.y() << " " << Candidate_AP.z() << " "  << std::endl;
      // std::cout << "Candidate_AP Index: " << i << std::endl;
      // std::cout << "Candidate_AP density: " << density_local_ << std::endl;

      Candidate_AP_density.push_back(density_local_);
      Candidate_AP_index.push_back(i);
    }
    else
    {
      // std::cout << "Candidate_AP position: " << Candidate_AP.x() << " " << Candidate_AP.y() << " " << Candidate_AP.z() << " "  << std::endl;
      // std::cout << "Candidate_AP Index: " << i << std::endl;
      // std::cout << "Candidate_AP density: " << 0 << std::endl;

      Candidate_AP_density.push_back(0);
      Candidate_AP_index.push_back(i);
    }

    density_local = 0;
  }
}

void Adjust_AP_Body_polyfit(const Eigen::Vector3d &AP_ref, Eigen::Vector3d &AP_k, int index)
{
  // Eigen::Vector3d
  std::vector<float> Candidate_AP_density;
  std::vector<float> Candidate_AP_index;

  // Circle_Sampling(AP_ref, Candidate_AP_density, Candidate_AP_index);
  Rectangle_Sampling(AP_ref, Candidate_AP_density, Candidate_AP_index);

  std::vector<float> Candidate_AP_density_normalize;

  float min_mid_val = 1000;
  float final_index = 0;

  for (int i = 0; i < Candidate_AP_density.size(); ++i)
  {
    alpha_distance = Candidate_AP_density[i] / Horizon_search_range; 

    // cout << "abs(Candidate_AP_index[i]): " << abs(Candidate_AP_index[i]) << endl;
    // cout << "Candidate_AP_density: " << Candidate_AP_density[i] << endl;

    double mid_val = -Candidate_AP_density[i] * alpha_density + abs(Candidate_AP_index[i]) * alpha_distance * 0.6;
    // cout << "mid_val : " << mid_val << endl;

    // 与参考轨迹点距离最近的最优
    if (mid_val <= min_mid_val)
    {
      final_index = Candidate_AP_index[i];
      min_mid_val = mid_val;
      // cout << "min_mid_val : " << min_mid_val << endl;
      // cout << "final_index : " << final_index << endl;
    }
  }

  float sum = 0;
  for (int i = 0; i < Candidate_AP_density.size(); ++i)
  {
    sum += Candidate_AP_density[i];
  }
  // cout << "sum " << sum << endl;

  if (abs(sum) > 0.001)
  {
    for (int i = 0; i < Candidate_AP_density.size(); ++i)
    {
      Candidate_AP_density_normalize.push_back((Candidate_AP_density[i] / sum) * 100);
    }
  }
  else
  {
    for (int i = 0; i < Candidate_AP_density.size(); ++i)
    {
      Candidate_AP_density_normalize.push_back(Candidate_AP_density[i]);
    }
  }

  // cout << "Candidate_AP_density_normalize: ";
  // for (int i = 0; i < Candidate_AP_density_normalize.size(); ++i)
  // {
  //   cout << " " << Candidate_AP_density_normalize[i];
  // }
  // cout << endl;

  Eigen::VectorXf result_ = FitterLeastSquareMethod(Candidate_AP_index, Candidate_AP_density_normalize, poly_order);
  for(int i = 0; i < result_.size(); ++i){
    polyfit_result[index][i] = result_(i);
  }

  // cout << "\nThe coefficients vector of " << index << " is: \n"
  //      << endl;
  // for (size_t i = 0; i < result_.size(); ++i)
  //   // cout << "index: " << index << "theta_" << i << ": " << polyfit_result[index][i] << endl;
  //   cout << ", " << result_(i);
  // cout << endl;

  // AP_k = AP_ref;
  AP_k = AP_ref + Dir_search * final_index * Octomap_Resolution_;

  AP_info.push_back(min_mid_val);
}

Eigen::Vector3d smoothnessTerm(Eigen::Vector3d p_b_2, Eigen::Vector3d p_b_, Eigen::Vector3d p_i, Eigen::Vector3d p_f_, Eigen::Vector3d p_f_2)
{
  // return wSmoothness_ * (-4) * (p_b_ - 2*p_i + p_f_);
  return (2) * (p_b_2 - p_b_ * 4 + 6 * p_i - 4 * p_f_ + p_f_2);
}

Eigen::Vector3d smoothnessTerm(Eigen::Vector3d p_b_, Eigen::Vector3d p_i, Eigen::Vector3d p_f_)
{
  return (-4) * (p_b_ - 2 * p_i + p_f_);
  // return wSmoothness_ * (2) * (p_b_2 - p_b_ * 4 + 6 * p_i - 4 * p_f_ + p_f_2);
}

// c_i: 某一行初始坐标; index: 行号
Eigen::Vector3d info_gradient_Term(float c_i, int index)
{
  // 在这里Anchor_point作为最佳观测参考点。
  Vector3d gradient(0, 0, 0);

  float gradient_y = 0; // 梯度

  if (c_i > 20)
  {
    c_i = 20;
  }
  if (c_i < -20)
  {
    c_i = -20;
  }

  for (int i = 1; i < poly_order + 1; ++i)
  {
    // cout << "polyfit_result[" << index - Anchor_index << "][" << i << "]:" << polyfit_result[index - Anchor_index][i] << endl;
    // cout << "pow(c_i, i-1):" << pow(c_i, i-1) << endl;
    gradient_y += polyfit_result[index - Anchor_index][i] * pow(c_i, i - 1) * i;
  }
  Circle_Index[index - Anchor_index] += wInfo_gradient_ * gradient_y; // 最后收敛值
  // cout << "c_i " << c_i << " Circle_Index[" << index - Anchor_index << "] " << Circle_Index[index - Anchor_index] << endl;

  if (index == 0)
  {
    cout << "initial value:" << c_i << endl;
    cout << "Circle_Index " << index << " :" << Circle_Index[index] << "================" << endl;
  }

  // 根据算出来的梯度恢复需要移动的距离
  Eigen::Vector3d Move_Vec(Dir_search * gradient_y * Octomap_Resolution_);
  // cout << "Move_Vec:" << Move_Vec.transpose() << endl;

  gradient = -wInfo_gradient_ * (Move_Vec);
  // cout << "gradient: " << gradient.transpose() << endl;
  return gradient;
}


Eigen::Vector3d MPC_Optimize_Gradient()
{
  // cout << "Enter optimize function........." << endl;
  int iterations = 0;

  vector<Eigen::Vector3d> Anchor_points_OptRef;
  for (int i = 0; i < Anchor_size - 1; ++i)
  {
    Anchor_points_OptRef.push_back(bp_position_sequence[i]);
  }

  // for(int i = 0; i < Anchor_size; ++i){
  //   Circle_Index[i] = 0;
  // }

  while (iterations < max_iterator_)
  {
    for (int i = 1; i < bp_position_sequence.size() - 1; ++i)
    {
      Eigen::Vector3d correction = Eigen::Vector3d::Zero(3, 1);

      if (i == 0)
      {
        // cout << "History optimize..." << endl;
        correction = correction - wSmoothness_ * smoothnessTerm(Sample_points[Anchor_index - 1], bp_position_sequence[i], bp_position_sequence[i + 1]);
      }
      else
      {
        correction = correction - wSmoothness_ * smoothnessTerm(bp_position_sequence[i - 1], bp_position_sequence[i], bp_position_sequence[i + 1]);
      }

      // cout << "correction smoothness: " << correction.transpose() << endl;

      bp_position_sequence[i] = bp_position_sequence[i] + correction;
    }

    for (int i = 0; i < bp_position_sequence.size() - 1; ++i)
    {
      Eigen::Vector3d correction = Eigen::Vector3d::Zero(3, 1);

      // Poly regression
      correction = correction - info_gradient_Term(Circle_Index[bp_index_sequence[i]], bp_index_sequence[i] + Anchor_index);
      // cout << "correction info_gradient_Term: " << correction.transpose() << endl;

      bp_position_sequence[i] = bp_position_sequence[i] + correction;
    }

    iterations++;

    // ROS_WARN_STREAM("iterations: " << iterations);
  }

  // for (int i = 0; i < bp_position_sequence.size() - 1; ++i)
  // {
  //   cout << "bp_position_sequence after optimize: " << bp_position_sequence[i].transpose() << endl;
  // }

  return bp_position_sequence[1];
}

// Build the local information map
void PathGrow()
{
  AP_info.clear();
  Candidate_AP_vec_g.clear();

  for (int i = Anchor_index; i < Anchor_index + Anchor_size; ++i)
  {
    Adjust_AP_Body_polyfit(Anchor_points[i - Anchor_index], Sample_points[i], i-Anchor_index);    // adjust Sample_points to best position

    // ROS_WARN_STREAM("Anchor_index: " << i);
    // ROS_WARN_STREAM("Sample_points: " << Sample_points[i].transpose());
    // ROS_WARN_STREAM("AP_info: " << AP_info[i-Anchor_index]);
    // ROS_WARN_STREAM("AP_info Length: " << AP_info.size());
  }
}

Eigen::Vector3d MPC_planning()
{

  Eigen::Vector3d last_bp;
  if (bp_position_sequence.empty() != true)
  {
    last_bp = bp_position_sequence[1];
  }

  bp_position_sequence.clear();
  bp_index_sequence.clear();

  for (int i = 0; i < Predict_step; ++i)
  {
    std::vector<float>::iterator min_value_tk = min_element(AP_info.begin() + i, AP_info.begin() + N_ap + i);
    int Index = std::distance(AP_info.begin(), min_value_tk);
    bp_index_sequence.push_back(Index);
    bp_position_sequence.push_back(Sample_points[bp_index_sequence[i] + Anchor_index]);
    // ROS_INFO_STREAM("min_value_points: " << Sample_points[Index + Anchor_index].transpose());
    // ROS_INFO_STREAM("min_value is : " << AP_info[Index]);
    // ROS_INFO_STREAM("min_value_tk[" << i << "] : " << Index);
    // ROS_INFO_STREAM("min_value_tk global index[" << i << "] : " << bp_index_sequence[i] + Anchor_index);
  }
  bp_position_sequence.insert(bp_position_sequence.begin(), last_bp);

  if (Anchor_index >= 2)
  {
    return MPC_Optimize_Gradient();
  }
  else
  {
    return bp_position_sequence[1];
  }
}

Eigen::Vector3d Single_Step_planning()
{
  bp_position_sequence.clear();

  std::vector<float>::iterator min_value_tk = min_element(AP_info.begin(), AP_info.begin() + 1);
  int Index = std::distance(AP_info.begin(), min_value_tk);
  ROS_INFO_STREAM("min_value_tk: " << Index);
  bp_position_sequence.push_back(Sample_points[Anchor_index + Index]); // for visualization

  return (Sample_points[Anchor_index + Index]);
}

Eigen::Vector3d run_IGO_ViewPlanner()
{
  current_position_.x() = g_T_w_c_igo(0, 3);
  current_position_.y() = g_T_w_c_igo(1, 3);
  current_position_.z() = g_T_w_c_igo(2, 3);

  Dir_search = Rot_90 * Body_dir_avg;
  ROS_INFO_STREAM("run_IGO_ViewPlanner......");

  PathGrow();   // sampling and regression

  Eigen::Vector3d Best_view_point_mpc;

  Best_view_point_mpc = MPC_planning();

  ROS_INFO_STREAM("Best_view_point_mpc:" << Best_view_point_mpc.transpose());

#if TIME_CONSUMING_COUNT
  second_sum += elapsed_seconds.count();
  second_count++;
  ROS_WARN_STREAM("Planning total time takes: " << second_sum << " ms");
  ROS_WARN_STREAM("Planning avergy time takes: " << second_sum / second_count << " ms");

  if(second_count%10 == 0){
    save_time_consuming("time_cus_proposed.txt", second_sum/second_count);
  }
#endif

  Dir_search = Rot_90 * Body_dir_avg;

  Body_dir_avg_last = Body_dir_avg;
  current_position_last = current_position_;

  Anchor_points_fixed[Anchor_index] = true;

  best_view_point_ = Best_view_point_mpc;

  Anchor_index++;

  Generate_TP_();

  int length_AP = Sample_points.size();
  Eigen::Vector3d Add_AP = Sample_points.back() + (Sample_points[length_AP - 1] - Sample_points[length_AP - 2]);

  Sample_points.push_back(Add_AP);
    
  return best_view_point_;
}
