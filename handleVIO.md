- [handleVIO](#handlevio)
  - [初始化图像帧数据结构`Frame`与图像帧位姿,重置网格grid信息](#初始化图像帧数据结构frame与图像帧位姿重置网格grid信息)
  - [`retrieveFromVisualSparseMap` 为视觉特征点寻找参考图像块，计算当前视觉帧与参考视觉帧的仿射变换，对图像块应用仿射变换](#retrievefromvisualsparsemap-为视觉特征点寻找参考图像块计算当前视觉帧与参考视觉帧的仿射变换对图像块应用仿射变换)
  - [`computeJacobianAndUpdateEKF` 基于LK光流和IEKF进行状态更新（不适用GN增量方程）](#computejacobianandupdateekf-基于lk光流和iekf进行状态更新不适用gn增量方程)
    - [`updateState(cv::Mat img, int level)` 迭代计算LK光流jacobian并进行IEKF更新状态](#updatestatecvmat-img-int-level-迭代计算lk光流jacobian并进行iekf更新状态)
    - [updateStateInverse(cv::Mat img, int level) 逆向LK光流](#updatestateinversecvmat-img-int-level-逆向lk光流)
    - [使用 kalman gain 更新状态协方差](#使用-kalman-gain-更新状态协方差)
  - [`generateVisualMapPoints` 生成视觉特征地图点](#generatevisualmappoints-生成视觉特征地图点)
    - [insertPointIntoVoxelMap 插入新视觉特征点到体素地图中](#insertpointintovoxelmap-插入新视觉特征点到体素地图中)
  - [plotTrackedPoints 绘制视觉子地图中的视觉特征点](#plottrackedpoints-绘制视觉子地图中的视觉特征点)
  - [projectPatchFromRefToCur 可视化当前帧与参考帧之间的图像块匹配结果（如光度误差、法向量投影等）](#projectpatchfromreftocur-可视化当前帧与参考帧之间的图像块匹配结果如光度误差法向量投影等)
  - [updateVisualMapPoints(cv::Mat img) 为视觉特征点添加新的图像观测帧信息](#updatevisualmappointscvmat-img-为视觉特征点添加新的图像观测帧信息)
  - [updateReferencePatch 检查视觉点是否收敛, 为每个视觉特征点更新其参考帧](#updatereferencepatch-检查视觉点是否收敛-为每个视觉特征点更新其参考帧)
  - [dumpDataForColmap() 保存colmap文件](#dumpdataforcolmap-保存colmap文件)


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

## `retrieveFromVisualSparseMap` 为视觉特征点寻找参考图像块，计算当前视觉帧与参考视觉帧的仿射变换，对图像块应用仿射变换

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
   6. 遍历图像块中的每一个像素点, 计算当前帧图像块（patch_buffer）与参考帧图像块（patch_wrap）之间的像素差值(考虑曝光时间的逆)`error`
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

## `computeJacobianAndUpdateEKF` 基于LK光流和IEKF进行状态更新（不适用GN增量方程）

若`total_points`为0,不执行，若启用了`inverse_composition_en`则使用逆向光流法计算光流，提高效率(这里和LIVO1 不同),调用`updateStateInverse(img, level)`,否则调用`updateState(img, level)`,最后利用迭代卡尔曼增益 G 对状态协方差矩阵进行修正，从而减小不确定性(P=(I-KH)P)，更新视觉帧状态`updateFrameState(*state)`

### `updateState(cv::Mat img, int level)` 迭代计算LK光流jacobian并进行IEKF更新状态

迭代计算以下内容

1. 从state中获取当前imu旋转平移并转换到camera,得到camera to world pose,计算位姿对平移向量求偏导(右扰动模型)

```cpp
    M3D Rwi(state->rot_end); // 当前帧imu的状态估计旋转
    V3D Pwi(state->pos_end); // 当前帧imu的状态估计平移
    Rcw = Rci * Rwi.transpose(); // 当前帧相机到世界坐标系的旋转
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci; // 当前帧相机到世界坐标系的平移
    Jdp_dt = Rci * Rwi.transpose(); // Pcw 对 Pwi 位姿对平移求偏导是R(右乘扰动)
```

2. 采用openMP对以下过程加速,计算整体jacobian矩阵 H_sub
   1. 基于state将视觉特征点转换到当前图像帧相机下并计算其像素坐标`V3D pf = Rcw * pt->pos_ + Pcw; V2D pc = cam->world2cam(pf); // 计算其像素坐标`
   2. 计算像素坐标对相机坐标的jacobian`computeProjectionJacobian` `Jdpi`
   3. 用双线性插值计算pc周围的像素梯度`du,dv`, 乘以逆曝光参数与缩放，得到归一化像素梯度`Jimg`
   4. 计算LK光流重投影模型jacobian`Jdphi` `Jdphi = Jimg * Jdpi * p_hat` 与对相机坐标的jacobian`Jdp = -Jimg * Jdpi;`
   5. 从投影空间转到imu空间`JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR; // 对旋转求偏导` `Jdt = Jdp * Jdp_dt; // 对平移求偏导` 用右乘扰动模型
   6. 在同一个图像块中计算每个像素的光度残差`res` (双线性插值加权后的像素值-仿射变换对齐的图像块像素值)
   7. 保存整体 H_sub jacobian矩阵`H_sub<<JdR, Jdt, cur_value`
3. 执行ESIKF流程，更新卡尔曼增益与state，检查光度残差的递减性`error`

可以注意到, 在以上LK光流中，每次迭代都是在当前帧新坐标点附近更新，需要重新计算jacobian和Hessian矩阵,为了提高计算效率，引入逆向光流

### updateStateInverse(cv::Mat img, int level) 逆向LK光流

jacobian矩阵只计算一次并没有伴随迭代更新计算

- `precomputeReferencePatches()` 将视觉特征点放在它的参考图像观测帧下，计算它们的jacobian(这里已经计算了像素梯度), 结果保存在`H_sub_inv`

在常规迭代中执行以下操作

1. 将视觉特征点从世界坐标系转到当前图像帧相机坐标系下并计算其像素坐标`pc`
2. 计算加权像素值，计算光度残差`res`(双线性插值加权后的像素值-仿射变换对齐的图像块像素值)
3. 在jacobian`H_sub`从投影空间转imu空间时会直接使用`H_sub_inv`而不是重新计算，这里提高了计算效率
4. 执行ESIKF流程，更新卡尔曼增益与state，检查光度残差的递减性`error`

### 使用 kalman gain 更新状态协方差

```cpp
  // 利用迭代卡尔曼增益 G 对状态协方差矩阵进行修正，从而减小不确定性
  // P=(I-KH)P
  state->cov -= G * state->cov;
  updateFrameState(*state);
```

## `generateVisualMapPoints` 生成视觉特征地图点

`VIOManager::generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg)`

1. 遍历lidar点云 pg , 筛选出具有高角点响应值的点，计算其图像网格id, 若该网格尚未被地图中的点占据`grid_num[index] != TYPE_MAP`, 计算该点的 Shi-Tomasi 角点响应值 cur_value，若大于该网格记录最大响应值则更新，并作以下处理

```cpp
   // 如果该网格尚未被地图中的点占据
   if (grid_num[index] != TYPE_MAP)
   {
     // 计算该点的 Shi-Tomasi 角点响应值 cur_value
     float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
     //如果 cur_value 高于该网格当前记录的最大响应值 scan_value[index] 则更新该网格响应值
     if (cur_value > scan_value[index])
     {
       scan_value[index] = cur_value;
       // 存储当前点 pg[i] 并标记该网格为 TYPE_POINTCLOUD
       append_voxel_points[index] = pg[i];
       grid_num[index] = TYPE_POINTCLOUD;
     }
   }
```

2. 从视觉子图 visual_submap 中筛选出具有高角点响应值的点，并将其加入到视觉地图中, 这些点通常是由光线投射 Raycasting 模块生成的候选特征点
   1. 遍历`visual_submap->add_from_voxel_map`,将点转为像素坐标计算Shi-Tomasi 角点响应值并更新对应网格属性`grid_num[index] = TYPE_POINTCLOUD;`
   2. `add_from_voxel_map`中的点也是来自lidar点云，只不过经过了Raycasting采样取近似点

```cpp
   int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);

   if (grid_num[index] != TYPE_MAP)
   {
     float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
     if (cur_value > scan_value[index])
     {
       scan_value[index] = cur_value;
       append_voxel_points[index] = visual_submap->add_from_voxel_map[j]; // 这里与第一部分不同
       grid_num[index] = TYPE_POINTCLOUD;
     }
   }
```

3. 对于每个`grid_num[index] = TYPE_POINTCLOUD`的网格
   1. 获取对应世界坐标点`pointWithVar pt_var = append_voxel_points[i]`，世界坐标`V3D pt = pt_var.point_w`, 转到当前图像帧相机,计算相机坐标(单位方向向量)`dir` 像素坐标`pc` 法向量`norm_vec`
   2. 计算方向向量与法向量的夹角余弦, 检查是否要反转法向量方向
   3. 提取以像素坐标 pc 为中心的图像块 patch，采用双线性插值方法处理亚像素精度
   4. 创建视觉特征点对象`VisualPoint *pt_new = new VisualPoint(pt)`与它的观测帧`Feature *ftr_new`
      1. 将观测帧加入该视觉特征点的观测历史`pt_new->addFrameRef(ftr_new)`
   5. 插入新视觉特征点到体素地图中`insertPointIntoVoxelMap(pt_new)`

### insertPointIntoVoxelMap 插入新视觉特征点到体素地图中

`void VIOManager::insertPointIntoVoxelMap(VisualPoint *pt_new)`

将点`pt_new`插入`feat_map`点云体素地图中

## plotTrackedPoints 绘制视觉子地图中的视觉特征点

优化后的残差小于原残差，对应视觉特征点显示为绿色，否则为蓝色

## projectPatchFromRefToCur 可视化当前帧与参考帧之间的图像块匹配结果（如光度误差、法向量投影等）

默认关闭

## updateVisualMapPoints(cv::Mat img) 为视觉特征点添加新的图像观测帧信息

对视觉子地图中的所有视觉特征点`visual_submap->voxel_points`作以下处理

1. 若该视觉特征点`pt`没有收敛则执行`pt->deleteNonRefPatchFeatures()`只保留该点的参考观测帧信息`ref_patch`, 否则跳过
2. 提取该点在当前图像帧`img`中的像素坐标 `pc`, 提取该点在当前帧中的图像块 `patch_temp`
3. 比较该视觉点上一帧观测与当前帧观测的位姿偏移 `delta_pose`
4. 如果相机移动较大或该点在图像上的运动超过一定阈值（如 40 像素）则标记为需更新 `add_flag = true`
5. 为该视觉特征点添加新的观测帧

```cpp
    if (add_flag)
    {
      // 添加新的观测帧
      update_num += 1;
      update_flag[i] = 1;
      Vector3d f = cam->cam2world(pc);
      Feature *ftr_new = new Feature(pt, patch_temp, pc, f, new_frame_->T_f_w_, visual_submap->search_levels[i]);
      ftr_new->img_ = img;
      ftr_new->id_ = new_frame_->id_;
      ftr_new->inv_expo_time_ = state->inv_expo_time;
      // 将观测帧 ftr_new 加入该特征点 pt 的观测历史
      pt->addFrameRef(ftr_new);
    }
```

## updateReferencePatch 检查视觉点是否收敛, 为每个视觉特征点更新其参考帧

`updateReferencePatch(feat_map),feat_map在函数替换为plane_map` 对视觉子地图中的所有视觉特征点`visual_submap->voxel_points`作以下处理

1. 对视觉特征点`VisualPoint *pt = visual_submap->voxel_points[i]`计算其空间体素位置并在`plane_map`中搜索最靠近`pt`的八叉树叶子节点`VoxelOctoTree *current_octo`
2. 若该节点具有平面模型，计算视觉特征点`pt`到该平面距离和`pt`到点云平面中心的距离平方，计算该点投影该点到平面上的距离`range_dis`，如果`range_dis<=3 * plane.radius_`
   1. 视觉点与平面模型不确定度(协方差)建模`sigma_l`
   2. 若视觉特征点`pt`到该平面距离`dis_to_plane_abs`小于`3 * sqrt(sigma_l)`
   3. 若法向量前后变化小且观测帧数量大于10则认为该视觉特征点已经收敛`pt->is_converged_ = true`
3. 对该视觉特征点`pt`所有的观测帧两两比较
   1. 获取`pt`在观测帧视角1下的图像块`ref_patch_temp`，并计算`pt`在观测帧视角1下的方向向量和法向量，然后计算该点图像块附近的平均灰度值`ref_mean`
   2. 计算视觉点`pt`在观测帧视角2下图像块`patch_cache`区域的灰度均值`other_mean`
   3. 计算这两个图像块之间的NCC,兼顾视角1下方向向量和法向量的夹角cos构造最终评分,那么这个评分兼顾光度一致性和几何一致性,作为视角1的评分
   4. 按照最高评分更新该视觉点的参考图像块`pt->ref_patch = ref_patch_temp`

## dumpDataForColmap() 保存colmap文件
