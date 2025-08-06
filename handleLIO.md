- [handleLIO](#handlelio)
  - [前置步骤](#前置步骤)
  - [LIO主要流程](#lio主要流程)
    - [lidar降采样与体素地图创建，构造八叉树地图](#lidar降采样与体素地图创建构造八叉树地图)
    - [基于ESIKF的状态估计](#基于esikf的状态估计)
    - [对新lidar点云更新体素地图`voxel_map_`](#对新lidar点云更新体素地图voxel_map_)
    - [体素点云滑窗](#体素点云滑窗)
    - [发布点云](#发布点云)

# handleLIO

主要步骤与fast-lio相似

## 前置步骤

1. `sync_packages()`-->`processImu()`
   1. imu初始化
   2. 基于eskf对imu进行误差状态推导(前向传播)
   3. lidar点云去畸变(后向传播)
   4. 重力方向与世界Z轴方向对齐，将状态量转到世界坐标系下
2. `stateEstimationAndMapping()`
   1. `handleLIO()`


## LIO主要流程

### lidar降采样与体素地图创建，构造八叉树地图

这个初始化点云体素地图只做一次，后续lidar点云直接走`StateEstimation`,故需要重新计算协方差

1. 对去畸变后的lidar点云做降采样，copy为lidar body坐标系系点云与世界坐标系点云到`voxelmap_manager->feats_down_body_` `voxelmap_manager->feats_down_world_`
2. (若是首次创建)初始化lidar点云体素地图`voxelmap_manager->BuildVoxelMap()`
   1. lidar body系下点云不确定性计算（每个点协方差矩阵计算）(测距不确定性+方位不确定性)
   2. world坐标系下点的协方差计算: 将body协方差从lidar坐标系转到imu, 在加上state状态估计的协方差，构成点的总协方差(`Efficient and Probabilistic Adaptive Voxel Mapping for Accurate`)
   3. 根据voxel_size计算每个点的体素id并建立八叉树体素地图`voxel_map_`
      1. 对体素点云进行平面模型估计，通过`init_octo_tree()`调用`init_plane(temp_points_, plane_ptr_)`

### 基于ESIKF的状态估计

`void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)`

1. 体素地图初始完后，后面新来的lidar点需要重新进行body协方差计算
2. 进入迭代误差卡尔曼流程
   1. 迭代次数i
      1. 计算点在世界坐标系下的总协方差
      2. lidar点，点到各自平面模型距离残差估计`BuildResidualListOMP`->`total_residual`
      3. 对所有lidar点云，计算观测方程对x_k(其实是对R T的jacobian)的jacobian矩阵，然后计算`hessian matrix = ``Hsub`
      4. ESIKF update, 计算HTz为`HTz为 H^T * R^-1 * -z` (H为jacobian, R为系统观测协方差，z为残差)，`H_T_H为H^T * R^-1 * H`
      5. 计算卡尔曼增益`(H^T * R^-1 * H + P)^-1 * H^T * R^-1`，这里用的高位观测等效处理方法
      6. 计算`vec 为 (x_predict ⊟ x_ki)`
      7. 更新最大后验估计，计算`\delta x_ki` `dx = -K * z - (x_ki ⊟ x_predict) + K * H * (x_ki ⊟ x_predict) 这是公式顺序`,转为`dx = -K * z + (x_predict ⊟ x_ki) - K * H * (x_predict ⊟ x_ki)`
      8. 更新最大后验估计(MAP)`x_k = x_pred ⊞ dx_ki`
      9. 如果旋转和平移的变化足够小，则认为 EKF 收敛，若不收敛则继续下一次迭代，直到到达最大迭代次数
   2. 更新状态变量协方差`P=(I-KH)*P_pred`
3. 状态估计完毕`_state = voxelmap_manager->state_`

### 对新lidar点云更新体素地图`voxel_map_`

- `UpdateVoxelMap` 将新增点添加到体素地图中 `void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)`
- `_pv_list = voxelmap_manager->pv_list_`,`_pv_list` 将用于handleVIO

### 体素点云滑窗

`void VoxelMapManager::mapSliding()` 当系统移动一定距离后，清理超出范围的体素地图数据，以保持地图在机器人周围的有效性并提升性能

### 发布点云

- `*pcl_w_wait_pub = *laserCloudWorld` pcl_w_wait_pub将用于`handleVIO`
