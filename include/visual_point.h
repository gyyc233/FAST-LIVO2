/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include "common_lib.h"
#include "frame.h"

class Feature;

// VisualPoint：代表一个 3D 地图点，并存储与该点相关的所有 图像特征观测（Feature），用于 VIO（Visual-Inertial Odometry）或 LIVO 中的优化、重投影误差计算和地图维护

/// A visual map point on the surface of the scene. 场景表面的可视化地图点
// 用于表示 场景中可视化的三维地图点（Visual Map Point）
class VisualPoint : boost::noncopyable
{
  // boost::noncopyable 不可拷贝类
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame. 3D点世界坐标
  Eigen::Vector3d normal_;             //!< Surface normal at point. 点的表面法向量
  Eigen::Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation. 法向量的信息矩阵
  Eigen::Vector3d previous_normal_;    //!< Last updated normal vector. 上一次更新的法向量
  list<Feature *> obs_;         //!< Reference patches which observe the point. 观测到该点的所有 reference patches
  Eigen::Matrix3d covariance_;  //!< Covariance of the point. 该3D地图点的协方差
  bool is_converged_;           //!< True if the point is converged. 是否收敛
  bool is_normal_initialized_;  //!< True if the normal is initialized. 法向量是否初始化
  bool has_ref_patch_;          //!< True if the point has a reference patch. 该点是否拥有参考patch
  Feature *ref_patch;           //!< Reference patch of the point. 该点的参考patch

  VisualPoint(const Vector3d &pos);
  ~VisualPoint();

  /// @brief 从当前地图点的所有视觉观测特征中，找到评分最低的那个特征（分数越低表示特征越清晰，质量越好）
  /// @param framepos 
  /// @param ftr 指向当前地图点中评分最低的 feature
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;

  /// @brief 删除当前地图点中所有非参考特征的feature观测，只保留一个作为参考patch
  void deleteNonRefPatchFeatures();

  void deleteFeatureRef(Feature *ftr);

  void addFrameRef(Feature *ftr);

  /// @brief 从当前地图点的所有观测特征中，找到一个“视角最接近”的 Feature（即与当前帧视角相似的参考特征）
  /// @param framepos 当前相机的世界坐标
  /// @param ftr 指向一个 Feature，它是该地图点中 视角最接近当前帧 的观测
  /// @param cur_px 当前图像上的像素坐标（未启用使用）
  /// @return 
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_
