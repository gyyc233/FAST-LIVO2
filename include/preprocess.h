/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

// LIDAR 数据预处理

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false) // 判断一个数的绝对值是否大于1e8

// lidar点的不同类型特征
enum LiDARFeature
{
  Nor, // normal point
  Poss_Plane, // possible plane point 可能是平面点
  Real_Plane, // real plane point
  Edge_Jump, // 在边缘处发生距离跳跃的点
  Edge_Plane, // 在平面边界上的点
  Wire, // 线状结构上的点，如电线等细长物体表面的点
  ZeroPoint // 零点或无效点
};

// 表示 LiDAR 点云中某个点的相邻方向
enum Surround
{
  Prev, // 当前点在扫描顺序中的前一个邻居点
  Next // 当前点在扫描顺序中的后一个邻居点
};

// 表示 LiDAR 点与其相邻点之间可能发生的边缘跳跃状态，通常用于边缘特征检测和点云结构分析
enum E_jump
{
  Nr_nor, // 正常无跳跃
  Nr_zero, // 零距离跳跃，可能是遮挡或无效点导致的异常
  Nr_180, // 180度方向变化，表示可能存在锐利边缘
  Nr_inf, // 无穷大跳跃，两点间距离差异极大，可能是断层或空洞边缘
  Nr_blind // 盲区跳跃，当前点处于传感器盲区附近，可能导致测量不稳定
};

// 存储 LiDAR 点云中每个点的附加信息和几何特征状态，不仅记录了点的基本几何属性，还包含了跳跃状态（边缘检测）和点的类别信息
struct orgtype
{
  double range; // 该点到原点的距离
  double dista; // 该点与下一个点的距离
  double angle[2]; // 用于表示与前后点之间的角度变化，辅助判断边缘或平面特性
  double intersect; // 表示交点相关的参数，可能用于平面交叉判断或线段相交分析
  E_jump edj[2]; // 跳跃状态数组，分别表示当前点相对于前一个（Prev）和后一个（Next）点的跳跃类型
  LiDARFeature ftype; // 当前点的几何特征类别，例如普通点（Nor）、平面点（Real_Plane）、边缘点（Edge_Jump）
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor; // 前后跳跃状态为无跳跃
    edj[Next] = Nr_nor;
    ftype = Nor; // 点特征为普通点
    intersect = 2;
  }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D; // PCL 提供的一个宏，用于添加 4D 坐标（x, y, z，以及对齐填充）
  float intensity; // 表示激光反射的强度，通常用于推断表面反射率
  float time; // 点的时间戳，通常相对于扫描开始的时间
  std::uint16_t ring; // 表示捕获该点的 LiDAR 传感器的环号（或通道）。对于多线 LiDAR 很有用
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros

// 这个宏将自定义的 velodyne_ros::Point 结构注册到 PCL 中
// 它将结构体字段（如 x, y, z, intensity, time, ring）映射到各自的类型和名称，使得 PCL 可以无缝处理这些点
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float timestamp;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))
/*****************/

// LiDAR 数据预处理
class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  /// @brief 处理来自 Livox 驱动的消息，并生成输出点云
  /// @param msg 
  /// @param pcl_out 输出点云
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  /// @brief 处理标准 ROS 点云消息（sensor_msgs::PointCloud2），并生成输出点云
  /// @param msg 
  /// @param pcl_out 
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full; // 完整点云
  PointCloudXYZI pl_corn; // 角点点云
  PointCloudXYZI pl_surf; // 表面点点云

  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar 最多128线激光缓冲区，最多支持128线LiDAR
  vector<orgtype> typess[128]; // maximum 128 line lidar 某个扫描线lidar点，每个点的附加信息

  int lidar_type; // LiDAR类型
  int point_filter_num; // 点云滤波数量
  int N_SCANS; // 扫描线数
  
  double blind; // 盲区距离
  double blind_sqr; // 盲区距离平方
  bool feature_enabled; // 是否启用特征提取
  bool given_offset_time; // 是否提供时间偏移

  ros::Publisher pub_full; // 发布完整点云
  ros::Publisher pub_surf; // 发布表面点点云
  ros::Publisher pub_corn; // 发布角点点云

private:
  // 处理 Livox 数据
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void xt32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void Pandar128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  // 为点云中的每个点分配几何特征，打标签
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);

  // 判断点是否为平面点
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

  // 检测小平面
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);

  // 判断点是否在边缘跳跃
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_