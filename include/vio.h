/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors; // 传播误差
  vector<float> errors; // residuals of image patches 基于LK光流,图像重投影误差模型创建的残差模型
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points; //稀疏地图中的视觉特征点
  vector<double> inv_expo_list; // 图像帧曝光的逆
  vector<pointWithVar> add_from_voxel_map; // 临时缓存的候选点列表，通常是通过 Raycasting 或其他方式从平面地图中提取的潜在特征点

  SubSparseMap()
  {
    // SIZE_LARGE default 500
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref; // 图像帧之间的仿射变换
  int search_level; // 图像金字塔层级
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

// 管理一个体素内包含的视觉特征点
class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points; // 属于该体素的所有视觉特征点
  int count; // 该体素内特征点数量
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

// 融合激光雷达（LiDAR）、惯性测量单元（IMU）和视觉
class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam; // 相机模型
  vk::PinholeCamera *pinhole_cam; //  针孔相机投影模型
  StatesGroup *state; // 当前状态变量
  StatesGroup *state_propagat; // 传播后的状态变量

  bool enable_vio; // 是否跳过vio,直接使用lio的先验进行图像对齐
  
  M3D Rli; // 传感器之间的旋转（下标 l:LiDAR; c:Camera; i:imu; w:world）
  M3D Rci; // 传感器之间的旋转（下标 l:LiDAR; c:Camera; i:imu; w:world）
  M3D Rcl; // 传感器之间的旋转（下标 l:LiDAR; c:Camera; i:imu; w:world）
  M3D Rcw; // 传感器之间的旋转（下标 l:LiDAR; c:Camera; i:imu; w:world）
  M3D Jdphi_dR;
  M3D Jdp_dt;
  M3D Jdp_dR; // jacobian matrix 右扰动
  V3D Pli; // 传感器之间的平移（下标 l:LiDAR; c:Camera; i:imu; w:world）
  V3D Pci; // 传感器之间的平移（下标 l:LiDAR; c:Camera; i:imu; w:world）
  V3D Pcl; // 传感器之间的平移（下标 l:LiDAR; c:Camera; i:imu; w:world）
  V3D Pcw; // 传感器之间的平移（下标 l:LiDAR; c:Camera; i:imu; w:world）

  vector<int> grid_num; // 视觉图像帧对应的网格id
  vector<int> map_index; // 保存对应网格id中点云到当前视觉图像帧相机的最短距离
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;

  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false; // 归一化互相关（NCC）
  bool colmap_output_en = false; // 导出 COLMAP 数据

  int width, height; // 图像尺寸
  int grid_n_width; // 一帧图像中的网格宽度
  int grid_n_height; // 一帧图像中的网格高度
  int length; // 网格总数
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size;
  int patch_size_total; // 每个图像块中包含的像素数
  int patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap; // 视觉子地图，使用体素与光线投射提取的视觉地图点
  std::vector<std::vector<V3D>> rays_with_sample_points; // 每个网格的采样点信息

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Matrix<double, DIM_STATE, DIM_STATE> G; // IEKF 迭代卡尔曼滤波的增益系数K
  Matrix<double, DIM_STATE, DIM_STATE> H_T_H; // IEKF jacobian 矩阵的逆乘以自身
  MatrixXd K;
  MatrixXd H_sub_inv; // 每个像素重投影模型对RT的jacobian矩阵，R分量在前，T在后

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map; // 全局视觉点体素地图
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; // 记录每个世界坐标系下lidar点云对应体素位置（这里是体素，不是网格）与标记情况
  unordered_map<int, Warp *> warp_map; // 帧之间仿射变换的映射表
  vector<VisualPoint *> retrieve_voxel_points; // 每个网格中最靠近相机的视觉特征点
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_; // 当前帧相机
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1, // 表示该网格已经被视觉地图中的点占据，不再考虑添加新的候选点
    TYPE_POINTCLOUD, // 潜在的高质量角点
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();

  // 基于逆向光流的图像重投影误差的IEKF状态更新，相比于传统LK光流优化了计算效率
  void updateStateInverse(cv::Mat img, int level);
  
  // 基于光流重投影误差和 IEKF 进行最小二乘状态更新
  void updateState(cv::Mat img, int level);

  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  
  /// @brief 从稀疏视觉地图中检索当前帧可见的点
  /// @param img 图像帧
  /// @param pg lidar 点云
  /// @param plane_map lidar 点云体素地图
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  
  /// @brief 对没有地图点占据的网格，依次在lidar点，光线投射采样点（保存在visual_submap->add_from_voxel_map）点集中选择高分的shiTomasi点，插入到 feat_map 体素地图中
  /// @param img 图像帧数据
  /// @param pg lidar data with covariance
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);

  /// @brief 设置imu到LiDAR的外参
  /// @note 取了rot的逆变换
  /// @param transl 
  /// @param rot 
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);

  /// @brief 设置LiDAR到相机的外参
  /// @param R 
  /// @param P 
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);

  /// @brief 实例化 `visual_submap`, 加载针孔相机内参，图像尺寸，计算camera到imu的外参,图像网格初始化
  /// @note 对于图像中的每一个网格，在其中心像素处沿视线方向（即相机光轴方向）生成一系列深度采样点 `rays_with_sample_points`
  void initializeVIO();

  /// @brief 从图像中提取以某个点为中心的图像块（patch），并使用双线性插值处理亚像素精度
  /// @param img input image
  /// @param pc image patch center
  /// @param patch_tmp 指向提取出的图像块数据
  /// @param level 图像金字塔层级
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);

  // 计算残差对预测投影点的偏导数，p是相机坐标系坐标（slam 14讲 p186）
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);

  /// @brief 基于LK光流和IEKF进行状态更新（不适用GN增量方程）
  /// @param img 
  void computeJacobianAndUpdateEKF(cv::Mat img);

  // 重置图像网格状态信息为下一帧数据准备
  void resetGrid();

  void updateVisualMapPoints(cv::Mat img);

  /// @brief 计算仿射矩阵，用于描述图像中某个特征点周围的小块（patch）在两个不同帧之间的形变，用于图像匹配
  /// @param cam 
  /// @param px_ref 参考帧中特征点的像素坐标
  /// @param f_ref 特征点的方向向量（单位向量），表示从相机原点指向该点的方向
  /// @param depth_ref 该特征点在参考帧下的深度值
  /// @param T_cur_ref 当前帧到参考帧的相对位姿变换
  /// @param level_ref 图像金字塔中参考帧所在的层级
  /// @param pyramid_level 
  /// @param halfpatch_size 图像块的一半大小
  /// @param A_cur_ref 输出的仿射变换矩阵
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);

  /// @brief 计算参考帧到当前帧之间的仿射变换矩阵，通过单应性投影实现
  /// @param cam camera model
  /// @param px_ref 参考帧中某个特征点的像素坐标
  /// @param xyz_ref 特征点对应的3D空间坐标
  /// @param normal_ref 特征点所在平面的法向量
  /// @param T_cur_ref 当前帧到参考帧的变换（SE(3)）
  /// @param level_ref 图像金字塔层级
  /// @param A_cur_ref 输出的仿射变换矩阵
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);

  /// @brief 通过仿射变换，将参考帧中的图像块“扭曲”成当前帧的样子
  /// @param A_cur_ref 从参考帧到当前帧的仿射变换矩阵
  /// @param img_ref 参考帧的图像
  /// @param px_ref 特征点在参考帧图像中的像素坐标
  /// @param level_ref 参考帧使用的金字塔层级
  /// @param search_level 根据图像块面积变化选择的搜索层级，用于控制在哪一层级进行匹配
  /// @param pyramid_level 当前处理的金字塔层级
  /// @param halfpatch_size 图像块的一半大小，用于定义图像块的范围
  /// @param patch 变形后的图像块
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);

  // 插入新特征点到体素地图
  void insertPointIntoVoxelMap(VisualPoint *pt_new);

  // 绘制跟踪点
  void plotTrackedPoints();

  void updateFrameState(StatesGroup state);

  /// @brief 可视化当前帧与参考帧之间的图像块匹配结果（如光度误差、法向量投影等）
  /// @param plane_map 
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

  /// @brief 为每个视觉特征点更新其参考帧
  /// @param plane_map 
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

  /// @brief 预计算视觉特征点在参考观测帧中图像块的重投影误差的jacobian matrix
  /// @param level 
  void precomputeReferencePatches(int level);

  //  导出 COLMAP 数据
  void dumpDataForColmap();

  /// @brief NCC Normalized Cross-Correlation 归一化互相关，衡量两个图像块之间的相似性
  /// @param ref_patch 参考图像快
  /// @param cur_patch 当前图像块
  /// @param patch_size 图像块中包含的像素数
  /// @return 
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);

  /// @brief 根据仿射变换矩阵的行列式值，决定在图像金字塔中应该使用哪一个层级来进行特征点匹配或图像块对齐
  /// @param A_cur_ref 从参考帧到当前帧的仿射变换矩阵
  /// @param max_level 图像金字塔的最大搜索层级
  /// @return 
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);

  /// @brief 双线性插值获取像素值
  /// @param img 三通道图像
  /// @param pc 想要获取像素值的亚像素位置
  /// @return 
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_