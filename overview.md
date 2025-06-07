
# LIVO2流程解析

LIVO2 主流程

1. LIVMapper mapper.initializeSubscribersAndPublishers()
   1. lidar 点云回调
      1. 点云预处理 PreprocessPtr p_pre->process(msg, ptr);
   2. imu 回调
      1. 检查imu与lidar时间对齐,ros_driver_fix_en强制硬同步，保存在 imu_buffer，prop_imu_buffer
   3. image 回调，缓存至 img_buffer
   4. 定时器 imu_prop_callback：
      1. 等待imu初始化，完成一次ekf位姿估计
      2. 若state_update_flg，基于prop_imu_buffer 数据，进行imu前向传播 prop_imu_once(imu_propagate, dt, acc_imu, omg_imu)，否则只对newest_imu进行传播，更新imu
      3. 提取更新后的imu_propagate发布到 `/LIVO2/imu_propagate`
