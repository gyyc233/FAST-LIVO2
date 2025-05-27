/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

// 为每个新创建的平面分配唯一id
static int voxel_plane_id = 0;

// 体素地图参数配置
typedef struct VoxelMapConfig
{
  double max_voxel_size_; // 体素网格最大尺寸
  int max_layer_; // 最大八叉树层级
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_; // 单个体素中允许的最大点数
  double planner_threshold_; // 平面检测阈值
  double beam_err_; // LiDAR 束间误差
  double dept_err_; // LiDAR 深度误差
  double sigma_num_; // 用于特征提取中的统计置信度
  bool is_pub_plane_map_;

  // config of local map sliding 滑动地图配置
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

// lidar点到对应拟合平面之间的关系
typedef struct PointToPlane
{
  Eigen::Vector3d point_b_; // 点在 body 坐标系下的位置
  Eigen::Vector3d point_w_; // 点在世界坐标系下的位置
  Eigen::Vector3d normal_; // 平面法向量
  Eigen::Vector3d center_; // 平面中心
  Eigen::Matrix<double, 6, 6> plane_var_; // 平面协方差矩阵
  M3D body_cov_;
  int layer_; // 所属八叉树层级
  double d_;
  double eigen_value_; // 特征值，用于判断是否为平面点
  bool is_valid_;
  float dis_to_plane_; // 点到平面的距离
} PointToPlane;

// 体素内拟合的平面参数
typedef struct VoxelPlane
{
  Eigen::Vector3d center_; // 平面中心点
  Eigen::Vector3d normal_; // 平面法向量
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_; // 平面协方差矩阵
  Eigen::Matrix<double, 6, 6> plane_var_; // 平面状态协方差
  float radius_ = 0; // 平面半径

  // 平面主成分分析
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;

  float d_ = 0; // 平面偏移量（Ax + By + Cz + d = 0）
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

// 点云数据被划分为多个 体素（Voxel）。每个体素由其 (x, y, z) 坐标唯一标识
// 体素的3D位置
class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// 自定义hash函数，用于快速查找三维空间中的体素位置
// 输入一个VOXEL_LOCATION，输出一个 int64_t 类型的哈希值，用于作为 unordered_map 的键索引
// 使用 std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> 可实现 O(1) 时间复杂度的体素查询
// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

// lidar单个点信息的简化信息
struct DS_POINT
{
  float xyz[3]; // location
  float intensity; // 强度
  int count = 0;
};

/// @brief 计算 LiDAR 点在 body 坐标系（LiDAR 原始点所在坐标系）下的协方差矩阵
/// @note 涉及激光点不确定度计算 https://arxiv.org/pdf/2103.01627
/// @param pb 
/// @param range_inc 距离测量标准差
/// @param degree_inc 角度测量误差
/// @param cov 协方差矩阵，表示该点的不确定性
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

// Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry
// https://arxiv.org/pdf/2103.01627.pdf

// 体素八叉树
class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_; // 当前节点保存的一组点云数据（带协方差）
  VoxelPlane *plane_ptr_; // 当前节点的拟合平面模型
  int layer_; // 当前节点所在八叉树层级
  int octo_state_; // 0 is end of tree, 1 is not 0表示叶子节点

  VoxelOctoTree *leaves_[8]; //当前节点的子节点指针数组

  double voxel_center_[3]; // x, y, z 体素中心
  std::vector<int> layer_init_num_;
  float quater_length_; // 体素边长的1/4
  float planer_threshold_; // 平面阈值
  int points_size_threshold_; // 触发子节点划分的最小点数阈值
  int update_size_threshold_; // 更新平面所需最小点数
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  /// @brief voxel octotree constructor
  /// @param max_layer 八叉树最大层级
  /// @param layer 当前层级
  /// @param points_size_threshold 划分子节点所需最小点数
  /// @param max_points_num 单个体素最多容纳点数
  /// @param planer_threshold 平面拟合阈值
  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }

  /// @brief 基于一组点计算主成分与协方差，拟合平面
  /// @param points 
  /// @param plane 
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);

  /// @brief 基于当前节点内点数量决定是否划分为8个子节点
  void init_octo_tree();

  void cut_octo_tree();

  /// @brief 将一个新的点插入到合适的子节点中，如果是叶子节点且未满，直接加入；否则递归查找或分裂
  /// @param pv 
  void UpdateOctoTree(const pointWithVar &pv);

  /// @brief 查找世界坐标pw所在的八叉树叶子节点
  /// @param pw 
  /// @return 
  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);

  /// @brief 插入点到八叉树中
  /// @param pv 
  /// @return 
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

// 管理体素地图，负责 LiDAR 点云插入、平面拟合、状态估计残差构建以及地图可视化等操作
class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_; // 体素地图配置参数
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_; // 体素地图

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;

  M3D extR_;
  V3D extT_;

  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;

  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);

  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  // 构建初始体素地图
  void BuildVoxelMap();

  V3F RGBFromVoxel(const V3D &input_point);

  /// @brief 将新点更新到现有体素地图中
  /// @param input_points 
  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  /// @brief 启用滑窗机制，根据当前位姿移动地图中心
  void mapSliding();

  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_