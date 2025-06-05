/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();

  // 初始化发布订阅
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  // 初始化组件
  void initializeComponents();
  // 初始化文件
  void initializeFiles();

  void run();

  // 重力对齐
  void gravityAlignment();
  // 首帧处理
  void handleFirstFrame();
  // 状态估计与地图匹配
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();

  /// @brief 保存pcl点云与(optional)colmap点云
  void savePCD();
  void processImu();
  
  // 同步 LiDAR 和 IMU 数据包
  bool sync_packages(LidarMeasureGroup &meas);
  
  // 单次imu状态传播
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);

  // 定时触发 IMU 状态传播，在系统状态更新后，IMU 数据对当前状态进行连续预测，为后续 LiDAR 处理提供先验位姿估计
  void imu_prop_callback(const ros::TimerEvent &e);

  /// @brief 将点坐标从lidar通过extR extT转到imu，再通过 rot, t 转到世界坐标系下
  /// @param rot imu基于世界坐标系旋转矩阵的状态估计
  /// @param t imu基于世界坐标系位移的状态估计
  /// @param input_cloud 
  /// @param trans_cloud 
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);

  // 标准点云回调（如 Ouster）
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);

  // Livox LiDAR 数据回调
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);

  // IMU 数据回调
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

  // 图像数据回调
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  // callback

  // 可视化发布
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  // 可视化发布

  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);

  // 获取图像帧数据
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_; // 运行模式
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map; // 存储地图的体素树结构
  
  string root_dir; // 系统根目录
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT; // lidar-imu 外参
  M3D extR; // lidar-imu 旋转

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05; // 地图分辨率
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0; // 陀螺仪，加计，曝光时间倒数协方差
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0; // 上一次传感器的时间戳
  double filter_size_surf_min = 0; // 点云降采样 leaf size
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;

  // 时间统计
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate;
  StatesGroup latest_ekf_state; // 最近一次EKF更新状态

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0; // 自定义时间偏移

  bool gravity_align_en = false; // 是否启用重力对齐
  bool gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0; // 曝光时间延迟
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0; // 自定义图像时间偏移
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer; // 原始LiDAR数据
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer; // imu 缓存
  deque<cv::Mat> img_buffer; // 图像帧缓冲
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;

  vector<double> extrinT;
  vector<double> extrinR;

  vector<double> cameraextrinT; // lidar-camera extrin translation
  vector<double> cameraextrinR; // lidar-camera extrin rotation
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map; // 当前可视化的子地图
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state; // 当前状态
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre; // lidar 点云预处理
  ImuProcessPtr p_imu; // imu处理
  VoxelMapManagerPtr voxelmap_manager; // 体素地图管理器
  VIOManagerPtr vio_manager;  // 视觉-惯性融合模块

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif