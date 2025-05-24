/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>

// 按curvature 从小到大排序，表示时间戳或时间偏移量（单位可能是毫秒），并非真正的曲率
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion 用于 imu 数据处理与点云运动补偿
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  // 设置lidar-imu坐标系变换
  void set_extrinsic(const V3D &transl, const M3D &rot);

  // 设置lidar-imu坐标系变换
  void set_extrinsic(const V3D &transl);

  // 设置lidar-imu坐标系变换
  void set_extrinsic(const MD(4, 4) & T);

  // 设置陀螺仪协方差
  void set_gyr_cov_scale(const V3D &scaler);

  // 设置加计协方差
  void set_acc_cov_scale(const V3D &scaler);

  // 设置陀螺仪bias协方差
  void set_gyr_bias_cov(const V3D &b_g);

  // 设置加计bias协方差
  void set_acc_bias_cov(const V3D &b_a);

  // 设置逆曝光时间的协方差
  void set_inv_expo_cov(const double &inv_expo);

  // 设置imu初始化阶段使用的帧数
  void set_imu_init_frame_num(const int &num);

  // 禁用imu
  void disable_imu();

  // 禁用重力估计
  void disable_gravity_est();

  // 禁用bias估计
  void disable_bias_est();

  // 禁用曝光时间估计
  void disable_exposure_est();

  // 结合LiDAR和IMU数据进行状态更新
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);

  // 去除LiDAR点云运动畸变
  // 通过结合 IMU 数据对点云进行前向传播和反向去畸变处理，确保点云在运动过程中保持一致性
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  ofstream fout_imu;
  double IMU_mean_acc_norm; // 初始化时imu平均加速度模长
  V3D unbiased_gyr; // 无偏陀螺仪

  V3D cov_acc; // 加计协方差
  V3D cov_gyr; // 陀螺仪协方差
  V3D cov_bias_gyr; // 陀螺仪偏置协方差
  V3D cov_bias_acc; // 加计偏置协方差
  double cov_inv_expo; // 相机逆曝光协方差
  double first_lidar_time; // 第一帧lidar时间
  bool imu_time_init = false; // imu时间初始化标志
  bool imu_need_init = true; // imu需要初始化标志
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type; // lidar类型

private:

  // imu静止初始化过程
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

  // 在没有 IMU 数据的情况下，对 LiDAR 点云进行前向传播（预测）和去畸变处理
  // 常用于系统启动阶段或 IMU 被禁用时
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  PointCloudXYZI pcl_wait_proc; // 点云缓冲区
  sensor_msgs::ImuConstPtr last_imu; // 上一次的imu数据
  PointCloudXYZI::Ptr cur_pcl_un_;
  vector<Pose6D> IMUpose; // 前向传播中保存的imu位姿

  M3D Lid_rot_to_IMU; // lidar to imu rotation
  V3D Lid_offset_to_IMU; // lidar to imu translation

  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last; // imu 前向传播角速度
  V3D acc_s_last; // imu 前向传播加速度
  double last_prop_end_time; // 上一帧传播的结束时间
  double time_last_scan; // 上一次scan扫描时间
  int init_iter_num = 1;
  int MAX_INI_COUNT = 20; // imu初始化阶段使用的最大帧数
  bool b_first_frame = true;

  bool imu_en = true; // imu禁用标志
  bool gravity_est_en = true; // 重力估计禁用标志
  bool ba_bg_est_en = true; // bias估计禁用
  bool exposure_estimate_en = true; // 相机逆曝光估计禁用
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif