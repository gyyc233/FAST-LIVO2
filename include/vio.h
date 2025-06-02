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
  vector<float> errors; // residuals
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points; //稀疏地图中的视觉特征点
  vector<double> inv_expo_list; // 图像帧曝光的逆
  vector<pointWithVar> add_from_voxel_map; // 从 voxel map 中添加的带有协方差信息的点

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
  
  M3D Rli, Rci, Rcl, Rcw; // 传感器之间的旋转
  M3D Jdphi_dR;
  M3D Jdp_dt;
  M3D Jdp_dR; // jacobian matrix 右扰动
  V3D Pli, Pci, Pcl, Pcw; // 传感器之间的平移

  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;

  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false; // 归一化互相关（NCC）
  bool colmap_output_en = false; // 导出 COLMAP 数据

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points; // 每个网格的采样点信息

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map; // 视觉点体素地图
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  unordered_map<int, Warp *> warp_map; // 帧之间仿射变换的映射表
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();

  // 反向状态更新
  void updateStateInverse(cv::Mat img, int level);
  
  // 状态更新
  void updateState(cv::Mat img, int level);

  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  
  /// @brief 生成视觉地图点
  /// @param img 
  /// @param pg 
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

  void initializeVIO();

  /// @brief 从图像中提取以某个点为中心的图像块（patch），并使用双线性插值处理亚像素精度
  /// @param img input image
  /// @param pc image patch center
  /// @param patch_tmp 指向提取出的图像块数据
  /// @param level 图像金字塔层级
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);

  // 计算残差对预测投影点的偏导数，p是相机坐标系坐标（slam 14讲 p186）
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);

  
  void computeJacobianAndUpdateEKF(cv::Mat img);

  void resetGrid();

  void updateVisualMapPoints(cv::Mat img);

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

  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);

  // 插入新特征点到体素地图
  void insertPointIntoVoxelMap(VisualPoint *pt_new);

  // 绘制跟踪点
  void plotTrackedPoints();

  void updateFrameState(StatesGroup state);

  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);

  void precomputeReferencePatches(int level);

  //  导出 COLMAP 数据
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);

  //  获取最佳搜索图像金字塔层级
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);

  // 双线性插值获取像素值
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