
# LIVO2流程解析

LIVO2 主流程

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
   1. sync_packages，组装数据到`LidarMeasureGroup &meas`->`LidarMeasures`
      1. ONLY_LIO：同步lidar点云与imu数据, 组装数据`MeasureGroup m` 到 `LidarMeasureGroup &meas`,meas.lio_vio_flg = LIO; 
      2. LIVO,该模式下，LIO比VIO优先
         1. VIO，组装lidar点云与imu数据，后`meas.lio_vio_flg = LIO;`，进入LIO分支
         2. LIO，组装图像帧数据，后`meas.lio_vio_flg = VIO;`，进入VIO分支
      3. ONLY_LO，仅使用lidar数据
   2. LIVMapper::handleFirstFrame() 首帧数据标志
   3. LIVMapper::processImu() 
      1. `ImuProcess p_imu->Process2(LidarMeasures, _state, feats_undistort);` 计算imu误差状态系数矩阵更新 _state，并进行lidar点云去畸变
      2. `Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);` 参考FAST-LIO 中的误差状态jacobian进行推导，如果没有imu数据则只对位姿与速度进行更新
      3. `ImuProcess::IMU_init` imu静止初始化
      4. `void ImuProcess::UndistortPcl` 参考FAST-LIO imu 误差状态转移方程 (ESKF)前向传播imu data,然后对lidar点云进行去畸变（后向传播）
      5. `gravityAlignment()` 重力方向对齐
      6. 将前向传播，后向传播的结果（状态向量与去畸变点云）保存到 `voxelmap_manager->state_ = _state;voxelmap_manager->feats_undistort_ = feats_undistort;`
   4. `LIVMapper::stateEstimationAndMapping() ` 基于`LidarMeasures.lio_vio_flg`
      1. VIO->`LIVMapper::handleVIO()`
      2. LIO and LO->`LIVMapper::handleLIO()`
      3. TODO: 
