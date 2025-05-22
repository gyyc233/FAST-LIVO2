/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_FEATURE_H_
#define LIVO_FEATURE_H_

#include "visual_point.h"

// A salient image region that is tracked across frames.
// 跨帧跟踪的显著图像区域
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType
  {
    CORNER, // 角点
    EDGELET // 边缘点
  };

  int id_; // 特征点ID
  FeatureType type_;     //!< Type can be corner or edgelet.
  cv::Mat img_;          //!< Image associated with the patch feature 图像patch
  Eigen::Vector2d px_;          //!< Coordinates in pixels on pyramid level 0. 在图像金字塔第 0 层的像素坐标
  Eigen::Vector3d f_;           //!< Unit-bearing vector of the patch feature. 图像patch的单位方位向量
  int level_;            //!< Image pyramid level where patch feature was extracted. 提取patch特征的图像金字塔级别
  VisualPoint *point_;   //!< Pointer to 3D point which corresponds to the patch feature. 3D 点对应的patch特征（视觉地图点）
  Eigen::Vector2d grad_;        //!< Dominant gradient direction for edglets, normalized. 边缘点的归一化后的主要梯度方向
  SE3 T_f_w_;            //!< Pose of the frame where the patch feature was extracted. 提取patch特征帧的位姿（base world）
  float *patch_;         //!< Pointer to the image patch data. 图像 patch 数据指针
  float score_;          //!< Score of the patch feature.
  float mean_;           //!< Mean intensity of the image patch feature, used for normalization. patch 的平均灰度值，用于亮度归一化
  double inv_expo_time_; //!< Inverse exposure time of the image where the patch feature was extracted. 图像曝光时间的倒数，用于光度一致性建模
  
  Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3 &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), T_f_w_(_T_f_w), mean_(0), score_(0), level_(_level), patch_(_patch), point_(_point)
  {
  }

  /// @brief 该特征所在的相机位置（即相机中心在世界坐标系中的位置）
  /// @return 
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
  
  ~Feature()
  {
    // ROS_WARN("The feature %d has been destructed.", id_);
    delete[] patch_;
  }
};

#endif // LIVO_FEATURE_H_
