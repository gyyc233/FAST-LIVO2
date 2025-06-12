- [handleVIO](#handlevio)
  - [初始化图像帧数据结构`Frame`与图像帧位姿,重置网格grid信息](#初始化图像帧数据结构frame与图像帧位姿重置网格grid信息)
  - [`retrieveFromVisualSparseMap`](#retrievefromvisualsparsemap)


# handleVIO

`void LIVMapper::handleVIO()`，lidar点云对应`pcl_w_wait_pub`,主要流程

```cpp
void VIOManager::processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time)
```

## 初始化图像帧数据结构`Frame`与图像帧位姿,重置网格grid信息

- 图像帧图像经过缩放后转灰度图，绑定相机内参，初始化Frame `new_frame_`
- `void VIOManager::updateFrameState(StatesGroup state)` 将imu基于世界坐标系的位姿转为相机基于世界坐标系的位姿

```cpp
void VIOManager::updateFrameState(StatesGroup state)
{
  std::cout<<"[ VIOManager::updateFrameState ] state:\n"<<state<<std::endl;
  M3D Rwi(state.rot_end);
  V3D Pwi(state.pos_end);
  Rcw = Rci * Rwi.transpose();
  Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
  new_frame_->T_f_w_ = SE3(Rcw, Pcw);// 当前帧相机位姿
  // std::cout<<"[ VIOManager::updateFrameState ] camera(SE_c_w) coordination new_frame_->T_f_w_(SE3:[R,T]):\n"<<new_frame_->T_f_w_<<std::endl;
}
```

- `void VIOManager::resetGrid()` 重置网格相关信息

## `retrieveFromVisualSparseMap`

函数`retrieveFromVisualSparseMap`基于当前处理逻辑点云帧（经时间对齐在原始点云帧基础上处理过的点云集合）中的点云集合信息动态提取当前帧可见的特征点，并构建高质量的局部子地图(visual_submap)，其中稀疏地图中的可视点会基于点的多个观测，选择最优的patch参考（不同点可能会选择不同的参考图像帧），并根据姿态变换对图像做出对应的放射变换以便后续的光度残差的计算，ESKF基于子地图的可视点的观测状态（光度残差）和IMU的位姿误差运动方程紧耦合优化位姿

TODO:
