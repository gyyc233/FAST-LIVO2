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

- `void VIOManager::resetGrid()` 重置网格相关信息, `total_points`置0

## `retrieveFromVisualSparseMap`

函数`retrieveFromVisualSparseMap`基于当前处理逻辑点云帧（经时间对齐在原始点云帧基础上处理过的点云集合）中的点云集合信息动态提取当前帧可见的特征点，并构建高质量的局部子地图(visual_submap)，其中稀疏地图中的可视点会基于点的多个观测，选择最优的patch参考（不同点可能会选择不同的参考图像帧），并根据姿态变换对图像做出对应的放射变换以便后续的光度残差的计算，ESKF基于子地图的可视点的观测状态（光度残差）和IMU的位姿误差运动方程紧耦合优化位姿

```cpp
/// @brief 从稀疏视觉地图中检索当前帧可见的点
/// @param img 图像帧
/// @param pg lidar 点云
/// @param plane_map lidar 点云体素地图
void VIOManager::retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
```
**首次进入VIO时,feat_map为空, total_points为0，会跳过 computeJacobianAndUpdateEKF(), 优先执行 generateVisualMapPoints()**

1. 对lidar 点云pg每个点，标记其在体素地图中的位置到`sub_feat_map`, 计算从世界坐标系`pt_w`转到当前数觉图像帧的相机坐标系`pt_c`并转为像素坐标`px`,若`px`处于图像内部则用此时`pt_c`的z作为当前图像帧该像素点的深度
2. 基于`sub_feat_map`,在`feat_map`全局视觉点体素地图中遍历，若在全局体素地图中发现`sub_feat_map`中的体素
   1. 遍历`feat_map`该视觉点体素地图的所有点云，计算这点云对应当前视觉图像帧的像素坐标，并转换为图像网格，定义该网格属性`grid_num[index] = TYPE_MAP` 表明该网格已有地图点可见，避免后续对该网格进行冗余的光线投射（raycasting）操作
   2. 遍历`feat_map`该视觉点体素地图的所有点云，按照图像网格id，计算它们到当前视觉图像帧相机的最短距离，并保存最短距离和对应视觉特征点`map_dist[index]` `retrieve_voxel_points[index]`
3. `feat_map`有时无法完全覆盖`grid_num`,若某些网格没有被点云地图点占据，则使用光线投射 Raycasting 方法
   1. 遍历`rays_with_sample_points`光线投射采样点，保存采样点转世界坐标`sample_point_w`, 跳过在`sub_feat_map`中出现的点
   2. 在`feat_map`中查找，过程基本与上一步一致，找出对应图像网格中距离当前视觉图像帧最近的视觉特征点id并保存数据
   3. `feat_map`不存在上述目标, 则在`plane_map`lidar点云体素地图中搜索最接近`sample_point_w`的节点`current_octo`，若`current_octo`具有自己的平面模型,执行`visual_submap->add_from_voxel_map.push_back(plane_center);`, 用于在后续生成视觉地图点
4. 筛选高质量视觉点，遍历所有网格
   1. 对于`grid_num[i] == TYPE_MAP`, 获取`VisualPoint *pt = retrieve_voxel_points[i]` 该网格中最靠近相机的视觉特征点,计算其相机坐标与像素坐标
      1. 计算该点从世界坐标系投影到相机坐标系下的深度`pt_cam[2]`与之前深度图中的深度(这里的深度从lidar点云中得到)`depth`比较，观察深度一致性，保留前后深度一致的点继续下一步处理
   2. 为视觉特征点寻找最合适的参考图像参考帧
      1. 若启用法向量计算
         1. 遍历该视觉特征点的所有观测数据`pt->obs_`, 计算两两视角下图像块之间的光度误差`photometric error`, 选择最小光度误差视角的图像块作为该视觉特征点的最佳参考帧`pt->ref_patch`
      2. 否则使用视角相似性`getCloseViewObs`（如角度差异和距离）来选择参考特征
         1. 对每个观测特征，获取其对应帧的相机中心位置，计算地图点看向该帧的方向向量，选择夹角最小的视角作为最佳参考帧
   3. 计算当前视觉帧与参考视觉帧的仿射变换`A_cur_ref_zero`
      1. 若启用法向量计算
         1. 将当前视觉点的法向量 pt->normal_ 和位置 pt->pos_ 从世界坐标系转换到参考帧（ref_ftr）的相机坐标系下
         2. 计算从参考帧（ref_ftr）到当前帧（new_frame_）的变换矩阵 `T_cur_ref`
         3. 根据参考帧像素坐标 **ref_ftr->px_、3D点坐标 pf、法向量 norm_vec 和帧间变换 T_cur_ref，计算仿射变换矩阵 A_cur_ref_zero**
         4. 根据仿射变换矩阵的行列式值判断需要在图像金字塔中的哪个层级进行搜索
      2. 否则若缓存中已有该参考帧对应的仿射变换信息（通过 warp_map 查找），则直接复用
   4. 遍历图像金字塔的不同层级（pyramid_level），对参考帧中的图像块进行变形操作
   5. 从当前帧图像中提取以 pc 点为中心的图像块，并使用双线性插值方法进行亚像素级别的采样，结果保存在 patch_buffer 中
   6. 遍历图像块中的每一个像素点, 计算当前帧图像块（patch_buffer）与参考帧图像块（patch_wrap）之间的像素差值
   7. 计算仿射变换后的图像块与当前帧图像块的归一化互相关NCC
   8. 保存数据

```cpp
   visual_submap->voxel_points.push_back(pt); // 将当前特征点 pt 加入到 visual_submap 的体素点列表中，表示这个点被成功追踪并用于建图
   visual_submap->propa_errors.push_back(error); // 存储当前点的传播误差（propagation error），用于评估追踪质量
   visual_submap->search_levels.push_back(search_level); // 记录在金字塔图像中进行匹配所使用的层级（level），用于后续调整搜索策略
   visual_submap->errors.push_back(error);
   visual_submap->warp_patch.push_back(patch_wrap); // 存储经过仿射变换后对齐的图像块（warped patch），可用于下一帧的追踪或光流计算
   visual_submap->inv_expo_list.push_back(ref_ftr->inv_expo_time_); // 存储参考帧的逆曝光时间（inverse exposure time），用于光度一致性补偿，避免不同光照条件下的误差放大
```

TODO:

