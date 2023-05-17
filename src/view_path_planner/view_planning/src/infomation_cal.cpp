#include <infomation_cal.h>

Eigen::Matrix4d g_T_w_c_igo = Eigen::Matrix4d::Identity();
double Octomap_Resolution_ = 0.4; // 分辨率
octomap::OcTree *maptree_igo_;
unordered_map<Node, int, NodeHash> density_map;

// 计算雅克比矩阵，用于费雪信息的计算
Matrix36 Cal_Jacobi(const Eigen::Vector3d &p_w, const Eigen::Matrix4d &T_c_w)
{
  Matrix34 T_c_w_34 = T_c_w.block<3, 4>(0, 0);

  Eigen::Vector4d p_w_41;
  p_w_41.block<3, 1>(0, 0) = p_w;
  p_w_41(3, 0) = 1;

  Eigen::Vector3d p_c = T_c_w_34 * p_w_41;
  // ROS_INFO_STREAM("Eigen::Vector3d : " << p_c.transpose());

  const double n = p_c.norm();
  Eigen::Matrix3d df_dpc = (1 / n) * Eigen::Matrix3d::Identity() -
                           (1 / (n * n * n)) * p_c * p_c.transpose();

  Matrix36 jac;
  jac.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d skew;

  skew.setZero();
  (skew)(0, 1) = -p_w[2];
  (skew)(1, 0) = p_w[2];
  (skew)(0, 2) = p_w[1];
  (skew)(2, 0) = -p_w[1];
  (skew)(1, 2) = -p_w[0];
  (skew)(2, 1) = p_w[0];

  jac.block<3, 3>(0, 3) = (-1.0) * skew;

  Matrix36 dpc_dse3 = T_c_w.block<3, 3>(0, 0) * jac;

  return df_dpc * dpc_dse3;
}

void save_time_consuming(const std::string file_name, float time)
{
  std::ofstream out_file(file_name, ios::app);
  if (out_file.is_open())
  {
    out_file << time << endl;
  }
  else
  {
    ROS_ERROR("Error open file...");
  }
}

const static Eigen::IOFormat EigenSpaceSeparatedFmt(
    Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

template <typename Type, int Rows, int Cols>
void save(const std::string file, const Eigen::Matrix<Type, Rows, Cols> &mat,
          const Eigen::IOFormat &io_format = EigenSpaceSeparatedFmt)
{
  std::ofstream out(file);
  // CHECK(out.is_open());
  if (out.is_open())
  {
    out << mat.format(io_format);
  }
  else
  {
    ROS_ERROR("Error open file");
  }
}

Eigen::Matrix4d Get_Twc_Infomation()
{
  Eigen::Matrix4d t_w_c = Eigen::Matrix4d::Identity();

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

  //////////////////////////////////////////////////////////////////////////////////////
  // Convert tf to matrix
  t_w_c.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d translate;
  translate[0] = Camera_World_Tf.getOrigin().getX();
  translate[1] = Camera_World_Tf.getOrigin().getY();
  translate[2] = Camera_World_Tf.getOrigin().getZ();
  t_w_c.block<3, 1>(0, 3) = translate;

  t_w_c.block<1, 4>(3, 0) << 0, 0, 0, 1;

  // ROS_INFO_STREAM("t_w_c : " << t_w_c);
  cout << "\n t_w_c =\n"
       << t_w_c << endl
       << endl;

  return t_w_c;
}

// Calculate the infomation of One cell
double Cal_Cell_Info(const Eigen::Vector3d &point, const Eigen::Matrix4d &t_w_c)
{
  Matrix36 J = Cal_Jacobi(point, t_w_c.inverse());

  // ROS_INFO_STREAM("Infomation Matrix : " << J);

  Matrix66 curr_info;
  curr_info = J.transpose() * J;
  double fisher_info = curr_info.trace();

  return fisher_info;
}

double Cal_Cell_Info_wUncertainty(const Eigen::Vector3d &point, const Eigen::Matrix4d &t_w_c)
{

  Matrix36 J = Cal_Jacobi(point, t_w_c.inverse());

  double distance_ = (point - t_w_c.block<3, 1>(0, 3)).norm();
  Matrix33 Q_z = Matrix33::Identity(3, 3);
  double delta_z = 0.001425 * distance_ * distance_;
  Q_z(0, 0) = 0.01;
  Q_z(1, 1) = 0.01;
  Q_z(2, 2) = delta_z;

  Q_z(0, 2) = (point - t_w_c.block<3, 1>(0, 3)).x() * (delta_z - 0.01);
  Q_z(1, 2) = (point - t_w_c.block<3, 1>(0, 3)).y() * (delta_z - 0.01);

  // ROS_INFO_STREAM("Infomation Matrix : " << J);

  Matrix66 curr_info;
  curr_info = J.transpose() * Q_z.inverse() * J;
  double fisher_info = curr_info.trace();

  return fisher_info;
}

Eigen::Vector3i Get_DnesityMap_Index(Eigen::Vector3d p)
{
  int index_x, index_y, index_z;
  if (p.x() > 0)
  {
    index_x = floor((p.x() + Octomap_Resolution_) / Octomap_Resolution_);
  }
  else if (p.x() < 0)
  {
    index_x = floor(p.x() / Octomap_Resolution_);
  }
  else
  {
    index_x = -1;
  }

  if (p.y() > 0)
  {
    index_y = floor((p.y() + Octomap_Resolution_) / Octomap_Resolution_);
  }
  else if (p.y() < 0)
  {
    index_y = floor(p.y() / Octomap_Resolution_);
  }
  else
  {
    index_y = -1;
  }

  if (p.z() > 0)
  {
    index_z = floor((p.z() + Octomap_Resolution_) / Octomap_Resolution_);
  }
  else if (p.z() < 0)
  {
    index_z = floor(p.z() / Octomap_Resolution_);
  }
  else
  {
    index_z = -1;
  }

  Eigen::Vector3i index(index_x, index_y, index_z);
  return index;
}

int Get_Point_Density(Eigen::Vector3d point)
{
  int density_val = 0;

  Eigen::Vector3i Index_Density = Get_DnesityMap_Index(point);

  auto res_search = density_map.find(Node(Index_Density.x(), Index_Density.y(), Index_Density.z()));
  if (res_search != density_map.end())
  {
    // std::cout << "search position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
    // std::cout << "search density: " << res_search->second << std::endl;

    density_val = res_search->second;
  }
  else
  {
    // std::cout << "search position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
    // std::cout << "search density: " << 0 << std::endl;
  }

  return density_val;
}

void get_neighbours(vector<Eigen::Vector3d> &neibor_pos, Eigen::Vector3d query_pos)
{
  Eigen::Vector3d neighbor_position;

  int N_extend = 1;
  // cout << "N_extend: " << N_extend << endl;

  for (int i = -N_extend; i <= N_extend; ++i)
  {
    for (int j = -N_extend; j <= N_extend; ++j)
    {
      for (int k = -N_extend; k <= N_extend; ++k)
      {
        // cout << "ijk " << i << " " << j << " " << k << endl;
        if (i == 0 && j == 0 && k == 0)
        {
          continue;
        }
        else
        {
          neighbor_position.x() = query_pos.x() + Octomap_Resolution_ * i;
          neighbor_position.y() = query_pos.y() + Octomap_Resolution_ * j;
          neighbor_position.z() = query_pos.z() + Octomap_Resolution_ * k;
          // cout << "distribution_Info - neighbor_position" << neighbor_position.transpose() << endl;
          neibor_pos.push_back(neighbor_position);
        }
      }
    }
  }
}

float Get_Point_distribution_Info(Eigen::Vector3d point)
{
  float _sum_neighbor = 0;
  float _mean_neighbor = 0;
  float _sum_deviation_neighbor = 0;
  float _deviation_neighbor = 0;

  vector<int> _neighbor_density_vector;

  int _counter = 0;

  Eigen::Vector3i Index_Density = Get_DnesityMap_Index(point);

  auto res_search = density_map.find(Node(Index_Density.x(), Index_Density.y(), Index_Density.z()));
  if (res_search != density_map.end())
  {
    // std::cout << "distribution_Info - center position: " << point.x() << " " << point.y() << " " << point.z() << " "  << std::endl;
    // std::cout << "distribution_Info - center density: " << res_search->second << std::endl;

    _sum_neighbor += res_search->second;
    _neighbor_density_vector.push_back(res_search->second);
    _counter++;
  }

  vector<Eigen::Vector3d> neighbors;
  get_neighbours(neighbors, point);

  for (int i = 0; i < neighbors.size(); ++i)
  {
    Eigen::Vector3i Index_Density = Get_DnesityMap_Index(neighbors[i]);

    auto res_search = density_map.find(Node(Index_Density.x(), Index_Density.y(), Index_Density.z()));
    if (res_search != density_map.end())
    {
      // std::cout << "distribution_Info - neighbor position: " << neighbors[i].x() << " " << neighbors[i].y() << " " << neighbors[i].z() << " "  << std::endl;
      // std::cout << "distribution_Info - neighbor density: " << res_search->second << std::endl;

      _sum_neighbor += res_search->second;
      _neighbor_density_vector.push_back(res_search->second);
      _counter++;
    }
  }

  if (_counter == 0)
  {
    return 0;
  }

  _mean_neighbor = _sum_neighbor / _counter;
  // std::cout << "distribution_Info - _mean_neighbor: " << _mean_neighbor << std::endl;

  for (int i = 0; i < _neighbor_density_vector.size(); ++i)
  {
    _sum_deviation_neighbor += pow((_neighbor_density_vector[i] - _mean_neighbor), 2);
  }

  _deviation_neighbor = sqrt(_sum_deviation_neighbor / _counter);
  // std::cout << "distribution_Info - _deviation_neighbor: " << _deviation_neighbor << std::endl;

  float _distribution_Info = _mean_neighbor * (1 + exp(-_deviation_neighbor));
  // std::cout << "distribution_Info - _distribution_Info: " << _distribution_Info << std::endl;

  return _distribution_Info;
}

double Dens_Thresh_info = 0.03; // 1000:only dense; 1:dense+info
double Get_Point_Infogain(Eigen::Vector3d point)
{
  double gain = 0.0;
  double gain_info = 1.0;
  float gain_dens = 1;

  gain_dens = Get_Point_distribution_Info(point);

  // Filter the outlier
  if (gain_dens >= Dens_Thresh_info)
  {
    octomap::point3d temp(point.x(), point.y(), point.z());

    octomap::OcTreeNode *node = maptree_igo_->search(temp);

    if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info(p_w_, g_T_w_c_igo);
    }
  }

  return gain_dens * gain_info;
}
// 考虑Information
double Get_Point_FisherInfogain(Eigen::Vector3d point)
{
  double gain = 0.0;
  double gain_info = 1.0;

  {
    octomap::point3d temp(point.x(), point.y(), point.z());

    octomap::OcTreeNode *node = maptree_igo_->search(temp);

    if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info(p_w_, g_T_w_c_igo);
      // std::cout << "search FisherInfo: " << gain_info << std::endl;
    }
  }

  return gain_info;
}

// 考虑Information和uncertainty
double Get_Point_FisherWunInfogain(Eigen::Vector3d point)
{
  double gain = 0.0;
  double gain_info = 1.0;

  {
    octomap::point3d temp(point.x(), point.y(), point.z());

    octomap::OcTreeNode *node = maptree_igo_->search(temp);

    if (node != NULL && maptree_igo_->isNodeOccupied(node))
    {
      Eigen::Vector3d p_w_(temp.x(), temp.y(), temp.z());

      gain_info = Cal_Cell_Info_wUncertainty(p_w_, g_T_w_c_igo);
      // std::cout << "search FisherInfo: " << gain_info << std::endl;
    }
  }

  return gain_info;
}

double Get_Max_z(Eigen::Vector3d point)
{
  double max_z = 0;
  int density_val = 0;
  int max_density = -100;

  int Z_depth = 10;
  Eigen::Vector3d point_inZ(point.x(), point.y(), point.z() + (Z_depth / 2) * Octomap_Resolution_);

  density_val = Get_Point_Density(point_inZ);

  for (int j = 0; j <= Z_depth; ++j)
  {
    if (density_val > max_density)
    {
      max_z = point_inZ.z();
      max_density = density_val;
      // cout << "max z position: " << point_inZ << endl;
      // cout << "max z axis: " << max_z << endl;
      // cout << "max_density: " << max_density << endl;
    }
    point_inZ.z() -= Octomap_Resolution_;
    density_val = Get_Point_Density(point_inZ);
  }

  return max_z;
}

int Get_Zaxis_Density(Eigen::Vector3d point)
{
  int density_val = 0;

  int Z_depth = 6;
  Eigen::Vector3d point_inZ(point.x(), point.y(), point.z() + (Z_depth / 2) * Octomap_Resolution_);

  density_val += Get_Point_Density(point_inZ);

  for (int j = 0; j <= Z_depth; ++j)
  {
    point_inZ.z() -= Octomap_Resolution_;
    density_val += Get_Point_Density(point_inZ);
  }

  return density_val;
}

float Get_Zaxis_distribution_Info(Eigen::Vector3d point)
{
  float density_val = 0;

  int Z_depth = 6;
  Eigen::Vector3d point_inZ(point.x(), point.y(), point.z() + (Z_depth / 2) * Octomap_Resolution_);

  density_val += Get_Point_distribution_Info(point_inZ);

  for (int j = 0; j <= Z_depth; ++j)
  {
    point_inZ.z() -= Octomap_Resolution_;
    density_val += Get_Point_distribution_Info(point_inZ);
  }

  return density_val;
}

int Get_Neighbor_Density(Eigen::Vector3d point)
{
  int density_val = 0;

  int Z_depth = 6;
  int X_depth = 1;
  int Y_depth = 1;

  for (int i = -X_depth; i <= X_depth; ++i)
  {
    for (int j = -Y_depth; j <= Y_depth; ++j)
    {
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, point.z() + (Z_depth / 2) * Octomap_Resolution_);

      density_val += Get_Point_Density(point_inZ);

      for (int j = 0; j <= Z_depth; ++j)
      {
        point_inZ.z() -= Octomap_Resolution_;
        density_val += Get_Point_Density(point_inZ);
      }
    }
  }
  return density_val;
}

float Get_Neighbor_Distribution(Eigen::Vector3d point)
{
  float density_val = 0;

  int Z_depth = 6;
  int X_depth = 1;
  int Y_depth = 1;

  for (int i = -X_depth; i <= X_depth; ++i)
  {
    for (int j = -Y_depth; j <= Y_depth; ++j)
    {
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, point.z() + (Z_depth / 2) * Octomap_Resolution_);

      density_val += Get_Point_distribution_Info(point_inZ);

      for (int j = 0; j <= Z_depth; ++j)
      {
        point_inZ.z() -= Octomap_Resolution_;
        density_val += Get_Point_distribution_Info(point_inZ);
      }
    }
  }
  return density_val;
}

float Get_Neighbor_Info(Eigen::Vector3d point)
{
  float density_val = 0;

  int Z_depth = 3;
  int X_depth = 0;
  int Y_depth = 0;

  for (int i = -X_depth; i <= X_depth; ++i)
  {
    for (int j = -Y_depth; j <= Y_depth; ++j)
    {
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, point.z() + (Z_depth / 2) * Octomap_Resolution_);

      density_val += Get_Point_Infogain(point_inZ);
      for (int j = 0; j <= Z_depth; ++j)
      {
        point_inZ.z() -= Octomap_Resolution_;
        density_val += Get_Point_Infogain(point_inZ);
      }
    }
  }

  return density_val;
}

float Get_Neighbor_FisherInfo(Eigen::Vector3d point)
{
  float density_val = 0;

  int Z_depth = 6;
  int X_depth = 1;
  int Y_depth = 1;

  float l_density_val = 0;
  float l_max_density = -100;
  double z_depth = point.z();
  point.z() = point.z() + (Z_depth / 2) * Octomap_Resolution_;
  // cout << "z_depth point: " << point << endl;

  for (int k = 0; k <= Z_depth; ++k)
  {
    l_density_val = Get_Point_Density(point);

    // cout << "z_depth l_density_val: " << l_density_val << endl;
    // cout << "z_depth point: " << point << endl;
    if (l_density_val > l_max_density)
    {
      l_max_density = l_density_val;
      z_depth = point.z();
      // cout << "z_depth l_max_density: " << l_max_density << endl;
      // cout << "z_depth best: " << z_depth << endl;
    }
    point.z() -= Octomap_Resolution_;
  }
  // cout << "z_depth best: " << z_depth << endl;

  if (l_max_density == 0)
  {
    return 0;
  }

  for (int i = -X_depth; i <= X_depth; ++i)
  {
    for (int j = -Y_depth; j <= Y_depth; ++j)
    {
      Eigen::Vector3d point_inZ(point.x() + i * Octomap_Resolution_, point.y() + j * Octomap_Resolution_, z_depth);

      density_val += Get_Point_FisherInfogain(point_inZ);

      // for(int k = 0; k <= Z_depth; ++k){
      //   point_inZ.z() -= Octomap_Resolution_;
      //   density_val += Get_Point_Infogain(point_inZ);
      // }
    }
  }
  return density_val;
}

void CalAllCell_Infos_Save(octomap::OcTree *map_tree_)
{

  std::vector<Eigen::Vector3d> point_pos_vec;
  std::vector<double> fisherinfo_save;
  std::vector<double> fisher_W_uncertainty_info_save;
  std::vector<double> densityinfo_save;
  std::vector<double> totalinfo_save;
  std::vector<double> totalinfo_W_un_save;

  int leafs_count = 0;

  // Eigen::Matrix4d t_w_c = Get_Twc_Infomation();
  Eigen::Matrix4d t_w_c = g_T_w_c_igo;

  int Z_depth = 6;

  for (octomap::OcTree::leaf_iterator it = map_tree_->begin_leafs(), end = map_tree_->end_leafs(); it != end; ++it)
  {
    if (map_tree_->isNodeOccupied(*it))
    {
      leafs_count++;

      octomap::point3d p = it.getCoordinate();
      // ROS_INFO_STREAM("Eigen::Vector3d : " << p);

      Eigen::Vector3d p_w_(p.x(), p.y(), p.z());

      if ((p_w_ - t_w_c.block<3, 1>(0, 3)).norm() < 1)
      {
        continue;
      }

      point_pos_vec.push_back(p_w_);

      double gain_total = 1.0;
      double gain_info = 1.0;
      double FWU_info = 1.0;
      float gain_dens = 1;

      // gain_dens = Get_Zaxis_Density(p_w_);
      gain_dens = Get_Zaxis_distribution_Info(p_w_);

      float density_val = 0;
      for (int j = -Z_depth / 2; j <= Z_depth / 2; ++j)
      {
        p_w_.z() = j * Octomap_Resolution_;
        density_val += Get_Point_FisherInfogain(p_w_);
      }
      gain_info = density_val;

      density_val = 0;
      for (int j = -Z_depth / 2; j <= Z_depth / 2; ++j)
      {
        p_w_.z() = j * Octomap_Resolution_;
        density_val += Get_Point_FisherWunInfogain(p_w_);
      }
      FWU_info = density_val;

      // ROS_INFO_STREAM("Fisher Infomation : " << fisher_info);

      fisherinfo_save.push_back(gain_info);
      fisher_W_uncertainty_info_save.push_back(FWU_info);
      densityinfo_save.push_back(gain_dens);

      gain_total = gain_dens * gain_info;
      totalinfo_save.push_back(gain_total);

      gain_total = gain_dens * FWU_info;
      totalinfo_W_un_save.push_back(gain_total);
    }
  }

  ROS_INFO_STREAM("leafs_count : " << leafs_count);

  Eigen::Vector3d translate = t_w_c.block<3, 1>(0, 3);
#if 0
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y");
  auto stCurrentTime = oss.str();

  Eigen::MatrixXd twc_mat = Eigen::Map<Eigen::Matrix3d>(translate.data()).transpose(); //转化成矩阵
  string filename = "/home/hitwzh/data_process/information/twc_" + stCurrentTime + ".txt";
  save(filename, twc_mat);

  Eigen::MatrixXd vox_pos_mat = Eigen::MatrixXd::Map(point_pos_vec[0].data(), 3, point_pos_vec.size());
  filename = "/home/hitwzh/data_process/information/position_" + stCurrentTime + ".txt";
  save(filename, vox_pos_mat);

  Eigen::MatrixXd totalinfo_mat = Eigen::VectorXd::Map(&totalinfo_save[0],totalinfo_save.size());//转化成矩阵
  filename = "/home/hitwzh/data_process/information/totalinfo_" + stCurrentTime + ".txt";
  save(filename, totalinfo_mat);

  Eigen::MatrixXd fisherinfo_mat = Eigen::VectorXd::Map(&fisherinfo_save[0], fisherinfo_save.size()); //转化成矩阵
  filename = "/home/hitwzh/data_process/information/fisher_" + stCurrentTime + ".txt";
  save(filename, fisherinfo_mat);

  Eigen::MatrixXd fisherWuninfo_mat = Eigen::VectorXd::Map(&fisher_W_uncertainty_info_save[0], fisher_W_uncertainty_info_save.size()); //转化成矩阵
  filename = "/home/hitwzh/data_process/information/fisher_w_Uncertain_" + stCurrentTime + ".txt";
  save(filename, fisherWuninfo_mat);

  Eigen::MatrixXd densityinfo_mat = Eigen::VectorXd::Map(&densityinfo_save[0],densityinfo_save.size());//转化成矩阵
  filename = "/home/hitwzh/data_process/information/density_" + stCurrentTime + ".txt";
  save(filename, densityinfo_mat);

  Eigen::MatrixXd totalinfo_W_un_mat = Eigen::VectorXd::Map(&totalinfo_W_un_save[0],totalinfo_W_un_save.size());//转化成矩阵
  filename = "/home/hitwzh/data_process/information/totalinfo_W_un_" + stCurrentTime + ".txt";
  save(filename, totalinfo_W_un_mat);

#endif
}

