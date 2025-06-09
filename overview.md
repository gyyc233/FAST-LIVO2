- [LIVO2流程解析](#livo2流程解析)
  - [LIVMapper 初始化](#livmapper-初始化)
  - [LIVMapper 最外层接口执行](#livmapper-最外层接口执行)
  - [](#)

# LIVO2流程解析

LIVO2 主流程

## LIVMapper 初始化

1. 从 ROS 参数服务器中读取一系列配置参数
2. LIVMapper::initializeComponents() 初始化 `vio_manager`
   1. 设置系统外参 `VIOManagerPtr vio_manager->setImuToLidarExtrinsic(extT, extR); // imu to lidar` `vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT); // lidar to camera`
   2. `VIOManager::initializeVIO()`: 实例化 `visual_submap`, 加载针孔相机内参，图像尺寸，计算camera到imu的外参,图像网格初始化
   3. `raycast_en` 对于图像中的每一个网格，在其中心像素处沿视线方向（即相机光轴方向）生成一系列深度采样点 `rays_with_sample_points`
   4. `ImuProcessPtr p_imu` 初始化 p_imu

## LIVMapper 最外层接口执行

1. LIVMapper mapper.initializeSubscribersAndPublishers()
   1. lidar 点云回调
      1. 点云预处理 `PreprocessPtr p_pre->process(msg, ptr);`, save to `lid_raw_data_buffer`
   2. imu 回调
      1. 检查imu与lidar时间对齐,ros_driver_fix_en强制硬同步，保存在 `imu_buffer`, `prop_imu_buffer`
   3. image 回调，缓存至 `img_buffer`
   4. 定时器 imu_prop_callback：
      1. 等待imu初始化，完成一次ekf位姿估计
      2. 若state_update_flg，基于prop_imu_buffer 数据，进行imu前向传播 prop_imu_once(imu_propagate, dt, acc_imu, omg_imu)，否则只对newest_imu进行传播，更新imu
      3. 提取更新后的imu_propagate发布到 `/LIVO2/imu_propagate`
2. LIVMapper::run() 
   1. `sync_packages()`，组装数据到`LidarMeasureGroup &meas`->`LidarMeasures`
      1. ONLY_LIO：同步lidar点云与imu数据, 组装数据`MeasureGroup m` 到 `LidarMeasureGroup &meas`,meas.lio_vio_flg = LIO; 
      2. LIVO,该模式下，LIO比VIO优先
         1. VIO，组装lidar点云与imu数据，后`meas.lio_vio_flg = LIO;`，进入LIO分支
         2. LIO，组装图像帧数据，后`meas.lio_vio_flg = VIO;`，进入VIO分支
      3. ONLY_LO，仅使用lidar数据
   2. `LIVMapper::handleFirstFrame()` 首帧数据标志
   3. `LIVMapper::processImu()`
      1. `ImuProcess p_imu->Process2(LidarMeasures, _state, feats_undistort);` 计算imu误差状态系数矩阵更新 _state，并进行lidar点云去畸变
      2. `Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);` 参考FAST-LIO 中的误差状态jacobian进行推导，如果没有imu数据则只对位姿与速度进行更新
      3. `ImuProcess::IMU_init` imu静止初始化
      4. `void ImuProcess::UndistortPcl` 参考FAST-LIO imu 误差状态转移方程 (ESKF)前向传播imu data,然后对lidar点云进行去畸变（后向传播）
      5. `gravityAlignment()` 重力方向对齐
      6. 将前向传播，后向传播的结果（状态向量与去畸变点云）保存到 `voxelmap_manager->state_ = _state;voxelmap_manager->feats_undistort_ = feats_undistort;`
   4. `LIVMapper::stateEstimationAndMapping() ` 基于`LidarMeasures.lio_vio_flg`执行LIO或者VIO流程
      1. VIO->`LIVMapper::handleVIO()`
         1. VIO 需要等待 `pcl_w_wait_pub` 数据，它在handleLIO中添加数据
         2. `VIOManager::processFrame` VIO主要流程
      2. LIO and LO->`LIVMapper::handleLIO()`
         1. lidar点云转世界坐标系 `voxelmap_manager->feats_down_world_`
         2. 创建lidar点云体素地图 `VoxelMapManager::BuildVoxelMap()` 对点云距离不确定度和方位不确定度建模，构建点云体素地图 `std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_`
         3. `VoxelMapManager::StateEstimation(StatesGroup &state_propagat)` ESIKF 迭代kalman 更新状态量
         4. 基于更新后的状态量更新lidar点云
         5. 更新体素地图 `VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)`
         6. 点云体素地图滑窗 `VoxelMapManager::mapSliding()` 当系统移动一定距离后，清理超出范围的体素地图数据，以保持地图在机器人周围的有效性并提升性能
   5. LIVMapper::savePCD() 保存pcl点云与(optional)colmap点云 `source: pcl_wait_save`

## 

TODO: 
