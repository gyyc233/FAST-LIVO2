#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <Eigen/Core>
#include <math.h>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

/// @brief 旋转向量通过 Roderigous Tranformation 指数映射 转换为旋转矩阵
/// @tparam T 
/// @param ang 旋转向量，向量表示旋转轴，模长表示旋转角度
/// @return 旋转矩阵
template <typename T> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 0.0000001)
  {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm; // 归一化旋转向量，得到旋转轴
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  else { return Eye3; }
}

/// @brief 将角速度与时间转为旋转矩阵
/// @tparam T 
/// @tparam Ts 
/// @param ang_vel 角速度向量（单位：弧度/秒）表示绕某轴旋转的速度
/// @param dt 
/// @return 经过 dt 秒后由该角速度产生的旋转矩阵
template <typename T, typename Ts> Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (ang_vel_norm > 0.0000001)
  {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;

    K << SKEW_SYM_MATRX(r_axis);

    T r_ang = ang_vel_norm * dt;

    /// Roderigous Tranformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  else { return Eye3; }
}

/// @brief 将轴角表示（axis-angle）的旋转转换为旋转矩阵，v1, v2, v3，分别代表旋转向量的 x、y、z 分量，模长代表旋转角度
/// @tparam T 
/// @param v1 
/// @param v2 
/// @param v3 
/// @return 
template <typename T> Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
  T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 0.00001)
  {
    T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> K;
    K << SKEW_SYM_MATRX(r_ang);

    /// Roderigous Tranformation
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  }
  else { return Eye3; }
}

/* Logrithm of a Rotation Matrix 旋转矩阵的对数映射*/
template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  // 计算旋转角 theta 如果 tr(R) 接近 3（如 2.999999），说明旋转角度非常小，直接设为 0
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/// @brief 旋转矩阵转欧拉角
/// @tparam T 
/// @param rot 
/// @return ZYX 欧拉角顺序（也叫 yaw-pitch-roll）
template <typename T> Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6; // 判断是否接近奇异状态（万向锁）
  T x, y, z;
  if (!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  }
  else
  {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

#endif
