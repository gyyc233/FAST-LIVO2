/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "vio.h"

VIOManager::VIOManager()
{
  // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
}

VIOManager::~VIOManager()
{
  delete visual_submap;
  for (auto& pair : warp_map) delete pair.second;
  warp_map.clear();
  for (auto& pair : feat_map) delete pair.second;
  feat_map.clear();
}

void VIOManager::setImuToLidarExtrinsic(const V3D &transl, const M3D &rot)
{
  // 这里旋转取了逆变换
  Pli = -rot.transpose() * transl;
  Rli = rot.transpose();
}

void VIOManager::setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P)
{
  // 
  Rcl << MAT_FROM_ARRAY(R);
  Pcl << VEC_FROM_ARRAY(P);
}

void VIOManager::initializeVIO()
{
  visual_submap = new SubSparseMap;

  fx = cam->fx();
  fy = cam->fy();
  cx = cam->cx();
  cy = cam->cy();
  image_resize_factor = cam->scale();

  printf("intrinsic: %.6lf, %.6lf, %.6lf, %.6lf\n", fx, fy, cx, cy);

  width = cam->width();
  height = cam->height();

  printf("width: %d, height: %d, scale: %f\n", width, height, image_resize_factor);

  // 估计camera到imu的外参
  Rci = Rcl * Rli;
  Pci = Rcl * Pli + Pcl;

  V3D Pic;
  M3D tmp;
  Jdphi_dR = Rci;
  Pic = -Rci.transpose() * Pci;
  tmp << SKEW_SYM_MATRX(Pic);
  // 向量对旋转Rci求偏导，右扰动更新
  Jdp_dR = -Rci * tmp;

  // grid_size 由配置文件给定
  // 图像划分为固定大小的网格（grid），grid_size 每个网格的像素尺寸
  // grid_n_width, grid_n_height：图像在宽和高方向上被划分的网格数量；length：总的网格数
  if (grid_size > 10)
  {
    grid_n_width = ceil(static_cast<double>(width / grid_size));
    grid_n_height = ceil(static_cast<double>(height / grid_size));
  }
  else
  {
    grid_size = static_cast<int>(height / grid_n_height);
    grid_n_height = ceil(static_cast<double>(height / grid_size));
    grid_n_width = ceil(static_cast<double>(width / grid_size));
  }
  length = grid_n_width * grid_n_height;

  // Raycasting on Demand 按需光线投射
  // 为每个图像网格（grid）生成一组沿着深度方向的采样点
  if(raycast_en)
  {
    // cv::Mat img_test = cv::Mat::zeros(height, width, CV_8UC1);
    // uchar* it = (uchar*)img_test.data;

    // 初始化边框标志数组，标记每个网格是否位于图像边缘
    border_flag.resize(length, 0);

    // 初始化每个网格的3D点采样信息
    std::vector<std::vector<V3D>>().swap(rays_with_sample_points);
    rays_with_sample_points.reserve(length);
    printf("grid_size: %d, grid_n_height: %d, grid_n_width: %d, length: %d\n", grid_size, grid_n_height, grid_n_width, length);

    float d_min = 0.1;
    float d_max = 3.0;
    float step = 0.2;
    for (int grid_row = 1; grid_row <= grid_n_height; grid_row++)
    {
      for (int grid_col = 1; grid_col <= grid_n_width; grid_col++)
      {
        std::vector<V3D> SamplePointsEachGrid; // 采样的3D点
        int index = (grid_row - 1) * grid_n_width + grid_col - 1;

        // 标记边界
        if (grid_row == 1 || grid_col == 1 || grid_row == grid_n_height || grid_col == grid_n_width) border_flag[index] = 1;

        // 计算网格对应像素
        int u = grid_size / 2 + (grid_col - 1) * grid_size;
        int v = grid_size / 2 + (grid_row - 1) * grid_size;
        // it[ u + v * width ] = 255;
        for (float d_temp = d_min; d_temp <= d_max; d_temp += step)
        {
          // 在d_min与d_max之间，每隔step深度采样，生成一个采样点
          V3D xyz;
          xyz = cam->cam2world(u, v); // 像素坐标转相机单位球面坐标
          xyz *= d_temp / xyz[2];
          // xyz[0] = (u - cx) / fx * d_temp;
          // xyz[1] = (v - cy) / fy * d_temp;
          // xyz[2] = d_temp;
          SamplePointsEachGrid.push_back(xyz);
        }
        rays_with_sample_points.push_back(SamplePointsEachGrid);
      }
    }
    // printf("rays_with_sample_points: %d, RaysWithSamplePointsCapacity: %d,
    // rays_with_sample_points[0].capacity(): %d, rays_with_sample_points[0]: %d\n",
    // rays_with_sample_points.size(), rays_with_sample_points.capacity(),
    // rays_with_sample_points[0].capacity(), rays_with_sample_points[0].size()); for
    // (const auto & it : rays_with_sample_points[0]) cout << it.transpose() << endl;
    // cv::imshow("img_test", img_test);
    // cv::waitKey(1);
  }

  if(colmap_output_en)
  {
    pinhole_cam = dynamic_cast<vk::PinholeCamera*>(cam);
    fout_colmap.open(DEBUG_FILE_DIR("Colmap/sparse/0/images.txt"), ios::out);
    fout_colmap << "# Image list with two lines of data per image:\n";
    fout_colmap << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    fout_colmap << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    fout_camera.open(DEBUG_FILE_DIR("Colmap/sparse/0/cameras.txt"), ios::out);
    fout_camera << "# Camera list with one line of data per camera:\n";
    fout_camera << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    fout_camera << "1 PINHOLE " << width << " " << height << " "
        << std::fixed << std::setprecision(6)  // 控制浮点数精度为10位
        << fx << " " << fy << " "
        << cx << " " << cy << std::endl;
    fout_camera.close();
  }
  grid_num.resize(length);
  map_index.resize(length);
  map_dist.resize(length);
  update_flag.resize(length);
  scan_value.resize(length);

  // patch_size 由文件给定，每一个patch包含patch_size_total个像素
  patch_size_total = patch_size * patch_size;
  patch_size_half = static_cast<int>(patch_size / 2);
  patch_buffer.resize(patch_size_total);
  warp_len = patch_size_total * patch_pyrimid_level; // patch_pyrimid_level 由文件配置
  border = (patch_size_half + 1) * (1 << patch_pyrimid_level);

  retrieve_voxel_points.reserve(length);
  append_voxel_points.reserve(length);

  sub_feat_map.clear();
}

void VIOManager::resetGrid()
{
  fill(grid_num.begin(), grid_num.end(), TYPE_UNKNOWN); // 视觉图像网格，标记每个网格类型为未知
  fill(map_index.begin(), map_index.end(), 0); // 每个网格对应的地图索引，初始为0
  fill(map_dist.begin(), map_dist.end(), 10000.0f); // 存储每个网格中最近地图点到相机的距离
  fill(update_flag.begin(), update_flag.end(), 0);
  fill(scan_value.begin(), scan_value.end(), 0.0f); // 每个网格中的角点响应值（Shi-Tomasi 分数），用于筛选高质量特征点

  retrieve_voxel_points.clear();
  retrieve_voxel_points.resize(length);

  append_voxel_points.clear();
  append_voxel_points.resize(length); // 用于存储新生成的候选点（如来自 Raycasting 或 LiDAR 点云）

  total_points = 0; // 记录当前帧中成功追踪的视觉点数量
}

// void VIOManager::resetRvizDisplay()
// {
  // sub_map_ray.clear();
  // sub_map_ray_fov.clear();
  // visual_sub_map_cur.clear();
  // visual_converged_point.clear();
  // map_cur_frame.clear();
  // sample_points.clear();
// }

// 计算残差对预测投影点的偏导数，p是相机坐标系坐标（slam 14讲 p186）
void VIOManager::computeProjectionJacobian(V3D p, MD(2, 3) & J)
{
  const double x = p[0];
  const double y = p[1];
  const double z_inv = 1. / p[2];
  const double z_inv_2 = z_inv * z_inv;

  // (预测-观测)
  J(0, 0) = fx * z_inv;
  J(0, 1) = 0.0;
  J(0, 2) = -fx * x * z_inv_2;
  J(1, 0) = 0.0;
  J(1, 1) = fy * z_inv;
  J(1, 2) = -fy * y * z_inv_2;
}

void VIOManager::getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level)
{
  const float u_ref = pc[0];
  const float v_ref = pc[1];
  const int scale = (1 << level);

  // 将图像块中心点 pc 的坐标转换为当前金字塔层级下的整数部分和小数部分 亚像素精度
  // 通过计算周围四个像素点的权重，并进行加权求和来获得目标位置的像素值
  const int u_ref_i = floorf(pc[0] / scale) * scale;
  const int v_ref_i = floorf(pc[1] / scale) * scale;
  const float subpix_u_ref = (u_ref - u_ref_i) / scale; // 特征点在 x 方向上的亚像素偏移量
  const float subpix_v_ref = (v_ref - v_ref_i) / scale; // 特征点在 y 方向上的亚像素偏移量

  // 计算了双线性插值的四个权重
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref); // 左上角像素点的权重
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref); // 右上角像素点的权重
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref; // 左下角像素点的权重
  const float w_ref_br = subpix_u_ref * subpix_v_ref; // 右下角像素点的权重

  for (int x = 0; x < patch_size; x++)
  {
    // 计算当前 patch 行在图像中的起始位置
    uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i - patch_size_half * scale + x * scale) * width + (u_ref_i - patch_size_half * scale);
    for (int y = 0; y < patch_size; y++, img_ptr += scale)
    {
      // 双线性插值 使用四个邻近像素（左上、右上、左下、右下）及其对应的权重进行插值计算
      patch_tmp[patch_size_total * level + x * patch_size + y] =
          w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] + w_ref_br * img_ptr[scale * width + scale];
    }
  }
}

void VIOManager::insertPointIntoVoxelMap(VisualPoint *pt_new)
{
  V3D pt_w(pt_new->pos_[0], pt_new->pos_[1], pt_new->pos_[2]); // 视觉点在世界坐标系下的坐标
  double voxel_size = 0.5; // 每个体素（voxel）的大小为 0.5 单位长度
  float loc_xyz[3]; // 对应体素坐标
  for (int j = 0; j < 3; j++)
  {
    // 将点的世界坐标除以 voxel_size，得到它在该维度上的体素索引
    loc_xyz[j] = pt_w[j] / voxel_size;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }

  // 插入点到体素地图
  VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
  auto iter = feat_map.find(position);
  if (iter != feat_map.end())
  {
    iter->second->voxel_points.push_back(pt_new);
    iter->second->count++;
  }
  else
  {
    // 如果没有找到对应的体素，则新建一个体素，并将当前视觉点添加进去
    VOXEL_POINTS *ot = new VOXEL_POINTS(0);
    ot->voxel_points.push_back(pt_new);
    feat_map[position] = ot;
  }
}

void VIOManager::getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref, const V3D &xyz_ref, const V3D &normal_ref,
                                                  const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref)
{
  // create homography matrix
  const V3D t = T_cur_ref.inverse().translation(); // 当前帧到参考帧的平移向量
  // 构造单应矩阵（因为特征点都在平面上）
  // TODO: 单应矩阵构造原理 https://blog.csdn.net/OrdinaryMatthew/article/details/121175440 下面式子还不包含相机内参，只是将空间点在两个相机坐标系之间进行转换
  const Eigen::Matrix3d H_cur_ref =
      T_cur_ref.rotation_matrix() * (normal_ref.dot(xyz_ref) * Eigen::Matrix3d::Identity() - t * normal_ref.transpose());
  // Compute affine warp matrix A_ref_cur using homography projection
  
  // 在参考帧图像上以 px_ref 为中心，分别沿x和y方向移动一定距离（kHalfPatchSize * scale），获取两个新的像素点
  // 将这两个像素点反投影到3D空间中，得到两个新的3D点 f_du_ref 和 f_dv_ref
  const int kHalfPatchSize = 4;
  V3D f_du_ref(cam.cam2world(px_ref + Eigen::Vector2d(kHalfPatchSize, 0) * (1 << level_ref)));
  V3D f_dv_ref(cam.cam2world(px_ref + Eigen::Vector2d(0, kHalfPatchSize) * (1 << level_ref)));
  //   f_du_ref = f_du_ref/f_du_ref[2];
  //   f_dv_ref = f_dv_ref/f_dv_ref[2];

  // 利用之前构造的单应性矩阵 H_cur_ref，将参考帧中的3D点 xyz_ref、f_du_ref 和 f_dv_ref 映射到当前帧中，得到相应的3D坐标
  const V3D f_cur(H_cur_ref * xyz_ref);
  const V3D f_du_cur = H_cur_ref * f_du_ref;
  const V3D f_dv_cur = H_cur_ref * f_dv_ref;

  // 将变换后的3D点投影回图像平面，得到它们在当前帧中的像素坐标，这里用了相机内参
  V2D px_cur(cam.world2cam(f_cur));
  V2D px_du_cur(cam.world2cam(f_du_cur));
  V2D px_dv_cur(cam.world2cam(f_dv_cur));

  // TODO: 将单应矩阵转为仿射变换矩阵
  // 通过比较当前帧与参考帧中对应点的位置差异，计算出仿射变换矩阵 A_cur_ref 的两列值
  A_cur_ref.col(0) = (px_du_cur - px_cur) / kHalfPatchSize;
  A_cur_ref.col(1) = (px_dv_cur - px_cur) / kHalfPatchSize;
  // 这个仿射变换矩阵可以用来对图像块进行仿射变换
}

void VIOManager::getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref,
                                        const SE3 &T_cur_ref, const int level_ref, const int pyramid_level, const int halfpatch_size,
                                        Matrix2d &A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const Vector3d xyz_ref(f_ref * depth_ref); // 根据点的方向向量f_ref与深度值depth_ref计算3D点（相机坐标系下）

  // 生成偏移点
  // 在原始图像中，以当前特征点为中心，在水平和垂直方向各偏移一定像素（考虑金字塔层级），然后反投影到3D空间
  Vector3d xyz_du_ref(cam.cam2world(px_ref + Vector2d(halfpatch_size, 0) * (1 << level_ref) * (1 << pyramid_level)));
  Vector3d xyz_dv_ref(cam.cam2world(px_ref + Vector2d(0, halfpatch_size) * (1 << level_ref) * (1 << pyramid_level)));
  // 调整偏移点的深度，使它们与中心点在同一深度平面上，确保几何一致性
  xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
  const Vector2d px_cur(cam.world2cam(T_cur_ref * (xyz_ref)));
  const Vector2d px_du(cam.world2cam(T_cur_ref * (xyz_du_ref)));
  const Vector2d px_dv(cam.world2cam(T_cur_ref * (xyz_dv_ref)));
  A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
  // A_cur_ref 这个矩阵描述了图像块如何被拉伸、旋转或剪切
}

void VIOManager::warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                               const int pyramid_level, const int halfpatch_size, float *patch)
{
  // 图像块通常是正方形，比如 halfpatch_size = 4 → 块大小为 8x8
  const int patch_size = halfpatch_size * 2;
  // 计算仿射矩阵的逆，因为是从参考帧映射到当前帧，需要反向
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  if (isnan(A_ref_cur(0, 0))) // 如果矩阵不可逆（如相机无运动），则报错并返回
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  float *patch_ptr = patch;
  // 遍历 image patch 中的每个像素点
  for (int y = 0; y < patch_size; ++y)
  {
    for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
    {
      Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
      // 每个像素相对于中心的偏移量乘上缩放因子
      px_patch *= (1 << search_level);
      px_patch *= (1 << pyramid_level);

      // 使用仿射矩阵将偏移量转换为参考帧中的像素
      const Vector2f px(A_ref_cur * px_patch + px_ref.cast<float>());
      // 若超出边界则为0，否则使用双线性插值获取像素值，更新变形后的图像块
      if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1)
        patch_ptr[patch_size_total * pyramid_level + y * patch_size + x] = 0;
      else
        patch_ptr[patch_size_total * pyramid_level + y * patch_size + x] = (float)vk::interpolateMat_8u(img_ref, px[0], px[1]);
    }
  }
}

int VIOManager::getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level)
{
  // Compute patch level in other image
  int search_level = 0;
  double D = A_cur_ref.determinant(); // 仿射变换行列式反映了图像块面积的变化倍数，越大表示需要更高的金字塔层级来缩小图像进行匹配
  while (D > 3.0 && search_level < max_level) // 当 D（面积变化）超过 3 倍时，就增加层级
  {
    search_level += 1;
    D *= 0.25; // 每提升一个金字塔层级，图像面积变为 1/4，每层*0.25是为了模拟这种效果
  }
  return search_level;
}

double VIOManager::calculateNCC(float *ref_patch, float *cur_patch, int patch_size)
{
  double sum_ref = std::accumulate(ref_patch, ref_patch + patch_size, 0.0);
  double mean_ref = sum_ref / patch_size;

  double sum_cur = std::accumulate(cur_patch, cur_patch + patch_size, 0.0);
  double mean_curr = sum_cur / patch_size;

  double numerator = 0, demoniator1 = 0, demoniator2 = 0;
  // NCC 公式12
  for (int i = 0; i < patch_size; i++)
  {
    double n = (ref_patch[i] - mean_ref) * (cur_patch[i] - mean_curr);
    numerator += n;
    demoniator1 += (ref_patch[i] - mean_ref) * (ref_patch[i] - mean_ref);
    demoniator2 += (cur_patch[i] - mean_curr) * (cur_patch[i] - mean_curr);
  }
  // 如果两个图像块非常相似，NCC 接近 1；若差异大，则接近 0 或负数
  return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);
}

void VIOManager::retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (feat_map.size() <= 0) return;
  double ts0 = omp_get_wtime();

  // pg_down->reserve(feat_map.size());
  // downSizeFilter.setInputCloud(pg);
  // downSizeFilter.filter(*pg_down);

  // resetRvizDisplay();
  visual_submap->reset();

  // Controls whether to include the visual submap from the previous frame.
  sub_feat_map.clear();

  float voxel_size = 0.5;

  if (!normal_en) warp_map.clear();

  // step 1. 初始化当前帧深度图，记录每个像素深度
  cv::Mat depth_img = cv::Mat::zeros(height, width, CV_32FC1);
  float *it = (float *)depth_img.data;

  // float it[height * width] = {0.0};

  // double t_insert, t_depth, t_position;
  // t_insert=t_depth=t_position=0;

  int loc_xyz[3];

  // printf("A0. initial depthmap: %.6lf \n", omp_get_wtime() - ts0);
  // double ts1 = omp_get_wtime();

  // printf("pg size: %zu \n", pg.size());

  for (int i = 0; i < pg.size(); i++)
  {
    // step 2. 将lidar点转到相机坐标系下
    // double t0 = omp_get_wtime();

    V3D pt_w = pg[i].point_w;

    for (int j = 0; j < 3; j++)
    {
      // step 3. 计算世界坐标pt_w所属的点云体素位置
      loc_xyz[j] = floor(pt_w[j] / voxel_size);
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

    // t_position += omp_get_wtime()-t0;
    // double t1 = omp_get_wtime();
    // 当前帧视野内涉及的体素列表（临时缓存）
    auto iter = sub_feat_map.find(position);
    if (iter == sub_feat_map.end()) { sub_feat_map[position] = 0; }
    else { iter->second = 0; }

    // t_insert += omp_get_wtime()-t1;
    // double t2 = omp_get_wtime();

    // 世界坐标转相机坐标
    V3D pt_c(new_frame_->w2f(pt_w));

    if (pt_c[2] > 0)
    {
      V2D px;
      // px[0] = fx * pt_c[0]/pt_c[2] + cx;
      // px[1] = fy * pt_c[1]/pt_c[2]+ cy;
      px = new_frame_->cam_->world2cam(pt_c);

      // 像素坐标是否在边界内部
      if (new_frame_->cam_->isInFrame(px.cast<int>(), border))
      {
        // cv::circle(img_cp, cv::Point2f(px[0], px[1]), 3, cv::Scalar(0, 0, 255), -1, 8);
        float depth = pt_c[2];
        int col = int(px[0]); // 列
        int row = int(px[1]); // 行
        it[width * row + col] = depth; // 该像素的深度
      }
    }
    // t_depth += omp_get_wtime()-t2;
  }

  // imshow("depth_img", depth_img);
  // printf("A1: %.6lf \n", omp_get_wtime() - ts1);
  // printf("A11. calculate pt position: %.6lf \n", t_position);
  // printf("A12. sub_postion.insert(position): %.6lf \n", t_insert);
  // printf("A13. generate depth map: %.6lf \n", t_depth);
  // printf("A. projection: %.6lf \n", omp_get_wtime() - ts0);

  // double t1 = omp_get_wtime();
  vector<VOXEL_LOCATION> DeleteKeyList;

  // 遍历当前帧视野中可能相关的体素
  for (auto &iter : sub_feat_map)
  {
    VOXEL_LOCATION position = iter.first;

    // double t4 = omp_get_wtime();
    // 获取该体素在全局地图中的位置
    auto corre_voxel = feat_map.find(position);
    // double t5 = omp_get_wtime();

    // 如果在全局地图中找到了这个体素，则继续处理，否则跳过
    if (corre_voxel != feat_map.end())
    {
      bool voxel_in_fov = false;
      // step 4. 在该点云体素中遍历所有视觉特征点，找到距离图像帧最近的视觉特征点，记录它与图像帧的最小深度，更新该网格为被点云地图点占据情况 TYPE_MAP
      // 该体素中的所有视觉特征点
      std::vector<VisualPoint *> &voxel_points = corre_voxel->second->voxel_points;
      int voxel_num = voxel_points.size();

      for (int i = 0; i < voxel_num; i++)
      {
        VisualPoint *pt = voxel_points[i];
        if (pt == nullptr) continue;
        if (pt->obs_.size() == 0) continue;

        // 视觉特征点的法向量转到当前帧坐标系下
        V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt->normal_);
        // 方向向量
        V3D dir(new_frame_->T_f_w_ * pt->pos_);
        if (dir[2] < 0) continue; // 跳过相机后方点
        // dir.normalize();
        // if (dir.dot(norm_vec) <= 0.17) continue; // 0.34 70 degree  0.17 80 degree 0.08 85 degree

        V2D pc(new_frame_->w2c(pt->pos_));
        if (new_frame_->cam_->isInFrame(pc.cast<int>(), border))
        {
          // cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 3, cv::Scalar(0, 255, 255), -1, 8);
          voxel_in_fov = true; // 该体素中的某个视觉点 pt 可见
          // 计算该点属于哪个网格
          int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);
          grid_num[index] = TYPE_MAP; // 表明该网格已有地图点可见，避免后续对该网格进行冗余的光线投射（raycasting）操作

          Vector3d obs_vec(new_frame_->pos() - pt->pos_); // 该点与当前帧相机的距离
          float cur_dist = obs_vec.norm();
          if (cur_dist <= map_dist[index])
          {
            map_dist[index] = cur_dist; // 存储最靠近相机的最小距离与最近视觉特征点
            retrieve_voxel_points[index] = pt;
          }
        }
      }
      if (!voxel_in_fov) { DeleteKeyList.push_back(position); }
    }
  }

  // step 5. 若某些网格没有被点云地图点占据，则使用 光线投射 Raycasting 方法
  // 对于没有地图点的网格，使用之前生成的采样点（rays_with_sample_points）进行光线投射，查找这些采样点是否落在 plane_map（来自 LiDAR 平面分割）中的平面上
  // 如果有有效平面，则将这些候选点加入 visual_submap->add_from_voxel_map
  // TODO: RayCasting Module 光线投射
  if (raycast_en)
  {
    for (int i = 0; i < length; i++)
    {
      // 若该网格点已经存在地图点，则跳过光线投射
      if (grid_num[i] == TYPE_MAP || border_flag[i] == 1) continue;

      // int row = static_cast<int>(i / grid_n_width) * grid_size + grid_size /
      // 2; int col = (i - static_cast<int>(i / grid_n_width) * grid_n_width) *
      // grid_size + grid_size / 2;

      // cv::circle(img_cp, cv::Point2f(col, row), 3, cv::Scalar(255, 255, 0),
      // -1, 8);

      // vector<V3D> sample_points_temp;
      // bool add_sample = false;

      // 对第i个网格里面的所有深度采样点（相机单位球面坐标）进行遍历
      for (const auto &it : rays_with_sample_points[i])
      {
        V3D sample_point_w = new_frame_->f2w(it); // 转世界坐标
        // sample_points_temp.push_back(sample_point_w);

        // 获取该点所在体素坐标
        for (int j = 0; j < 3; j++)
        {
          loc_xyz[j] = floor(sample_point_w[j] / voxel_size);
          if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }

        VOXEL_LOCATION sample_pos(loc_xyz[0], loc_xyz[1], loc_xyz[2]);

        // 如果该体素已经在 sub_feat_map 中存在（即当前帧已经处理过）则跳过
        auto corre_sub_feat_map = sub_feat_map.find(sample_pos);
        if (corre_sub_feat_map != sub_feat_map.end()) break;

        // step 6. 果在全局地图 feat_map 中找到了这个体素，则进入后续逻辑继续检查其中的视觉采样点
        auto corre_feat_map = feat_map.find(sample_pos);
        if (corre_feat_map != feat_map.end())
        {
          bool voxel_in_fov = false;

          // 属于该体素的所有视觉特征点
          std::vector<VisualPoint *> &voxel_points = corre_feat_map->second->voxel_points;
          int voxel_num = voxel_points.size();
          if (voxel_num == 0) continue;

          for (int j = 0; j < voxel_num; j++)
          {
            // 每个pt 包含其位置 pos_ 和法向量 normal
            VisualPoint *pt = voxel_points[j];

            if (pt == nullptr) continue;
            if (pt->obs_.size() == 0) continue; // 该点没有观测记录，无法用于匹配

            // sub_map_ray.push_back(pt); // cloud_visual_sub_map
            // add_sample = true;

            // 视觉特征点的法向量和方向向量转到当前帧坐标系下
            V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt->normal_);
            V3D dir(new_frame_->T_f_w_ * pt->pos_);
            if (dir[2] < 0) continue;
            dir.normalize();
            // if (dir.dot(norm_vec) <= 0.17) continue; // 0.34 70 degree 0.17 80 degree 0.08 85 degree

            // 转到像素坐标
            V2D pc(new_frame_->w2c(pt->pos_));

            if (new_frame_->cam_->isInFrame(pc.cast<int>(), border))
            {
              // cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 3, cv::Scalar(255, 255, 0), -1, 8); 
              // sub_map_ray_fov.push_back(pt);

              voxel_in_fov = true;
              // 计算该点所属的图像网格id
              int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);
              grid_num[index] = TYPE_MAP; // 表明该网格已有有效地图点
              Vector3d obs_vec(new_frame_->pos() - pt->pos_); // 当前帧相机位置到该点的距离向量

              float cur_dist = obs_vec.norm();

              // 网格中最靠近相机的视觉特征点
              if (cur_dist <= map_dist[index])
              {
                map_dist[index] = cur_dist;
                retrieve_voxel_points[index] = pt;
              }
            }
          }

          if (voxel_in_fov) sub_feat_map[sample_pos] = 0;
          break;
        }
        // step 7. 全局视觉点体素地图不存在该体素
        // 在某些区域没有视觉点时，通过 raycasting 从平面地图中提取候选点
        else
        {
          // 在 plane_map 中查找该体素
          VOXEL_LOCATION sample_pos(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
          auto iter = plane_map.find(sample_pos);
          if (iter != plane_map.end())
          {
            VoxelOctoTree *current_octo;
            // 查找该体素中与世界坐标 sample_point_w 最接近的节点
            current_octo = iter->second->find_correspond(sample_point_w);
            if (current_octo->plane_ptr_->is_plane_)
            {
              pointWithVar plane_center;
              VoxelPlane &plane = *current_octo->plane_ptr_;
              plane_center.point_w = plane.center_;
              plane_center.normal = plane.normal_;
              visual_submap->add_from_voxel_map.push_back(plane_center);
              break;
            }
          }
        }
      }
      // if(add_sample) sample_points.push_back(sample_points_temp);
    }
  }

  // 删除无效体素
  for (auto &key : DeleteKeyList)
  {
    sub_feat_map.erase(key);
  }

  // double t2 = omp_get_wtime();

  // cout<<"B. feat_map.find: "<<t2-t1<<endl;

  // double t_2, t_3, t_4, t_5;
  // t_2=t_3=t_4=t_5=0;

  // step 8. 筛选高质量视觉点

  // 遍历所有网格
  for (int i = 0; i < length; i++)
  {
    if (grid_num[i] == TYPE_MAP)
    {
      // double t_1 = omp_get_wtime();

      VisualPoint *pt = retrieve_voxel_points[i]; // 该网格中最靠近相机的视觉特征点
      // visual_sub_map_cur.push_back(pt); // before

      V2D pc(new_frame_->w2c(pt->pos_));

      // cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 3, cv::Scalar(0, 0, 255), -1, 8); // Green Sparse Align tracked

      V3D pt_cam(new_frame_->w2f(pt->pos_));
      bool depth_continous = false; // 标记该点是否与局部区域的深度值一致

      // 检查该点周围是否有剧烈的深度跳跃
      for (int u = -patch_size_half; u <= patch_size_half; u++)
      {
        for (int v = -patch_size_half; v <= patch_size_half; v++)
        {
          if (u == 0 && v == 0) continue;

          // 获取pc周围像素的深度
          float depth = it[width * (v + int(pc[1])) + u + int(pc[0])];

          if (depth == 0.) continue;

          double delta_dist = abs(pt_cam[2] - depth);

          // 任何一点的深度差大于0.5
          if (delta_dist > 0.5)
          {
            depth_continous = true;
            break;
          }
        }
        // 如果检测到深度跳跃，则认为这个点可能不可靠，丢弃处理
        if (depth_continous) break;
      }
      if (depth_continous) continue;

      // t_2 += omp_get_wtime() - t_1;

      // t_1 = omp_get_wtime();
      Feature *ref_ftr;
      std::vector<float> patch_wrap(warp_len);

      int search_level;
      Matrix2d A_cur_ref_zero;

      // 为一个3D点 pt 选择最合适的参考图像特征（ref_ftr）
      // 选择最佳参考帧（ref_patch）

      if (!pt->is_normal_initialized_) continue;

      // 如果启用了法向量信息，则通过比较不同视角下图像块的光度误差来选择最稳定的参考特征
      if (normal_en)
      {
        float phtometric_errors_min = std::numeric_limits<float>::max();

        // 如果只有一条观测数据（pt->obs_.size() == 1），就直接把它作为参考特征
        if (pt->obs_.size() == 1)
        {
          ref_ftr = *pt->obs_.begin();
          pt->ref_patch = ref_ftr;
          pt->has_ref_patch_ = true;
        }
        else if (!pt->has_ref_patch_)
        {
          // 遍历所有观测数据
          for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
          {
            Feature *ref_patch_temp = *it;
            float *patch_temp = ref_patch_temp->patch_; // 获取该观测视角的图像块，视角1
            float phtometric_errors = 0.0; // 用于累加当前观测视角与其他视角之间的光度误差
            int count = 0;

            // 两两观测视角对比
            for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme; ++itm)
            {
              // 自己和自己比则跳过
              if ((*itm)->id_ == ref_patch_temp->id_) continue;
              float *patch_cache = (*itm)->patch_; // 获取视角2图像块

              for (int ind = 0; ind < patch_size_total; ind++)
              {
                // 计算两个不同视角下图像块之间的光度误差（photometric error），即每个像素差值的平方之和
                phtometric_errors += (patch_temp[ind] - patch_cache[ind]) * (patch_temp[ind] - patch_cache[ind]);
              }
              count++;
            }
            // 计算平均两个视角的平均光度误差
            phtometric_errors = phtometric_errors / count;
            // 更新最小误差并将该视角作为最佳参考帧
            if (phtometric_errors < phtometric_errors_min)
            {
              phtometric_errors_min = phtometric_errors;
              ref_ftr = ref_patch_temp;
            }
          }

          // 为pt选择平均光度误差最小的图像块作为最佳参考帧
          pt->ref_patch = ref_ftr;
          pt->has_ref_patch_ = true;
        }
        else { ref_ftr = pt->ref_patch; }
      }
      // 如果没有启用法向量信息，则视角相似性（如角度差异和距离）来选择参考特征
      else
      {
        if (!pt->getCloseViewObs(new_frame_->pos(), ref_ftr, pc)) continue;
      }

      // step 9. 计算当前视觉帧与参考视觉帧的仿射变换
      if (normal_en)
      {
        // 将当前视觉点的法向量 pt->normal_ 和位置 pt->pos_ 从世界坐标系转换到参考帧（ref_ftr）的相机坐标系下
        V3D norm_vec = (ref_ftr->T_f_w_.rotation_matrix() * pt->normal_).normalized();
        V3D pf(ref_ftr->T_f_w_ * pt->pos_);
        // V3D pf_norm = pf.normalized();
        
        // double cos_theta = norm_vec.dot(pf_norm);
        // if(cos_theta < 0) norm_vec = -norm_vec;
        // if (abs(cos_theta) < 0.08) continue; // 0.5 60 degree 0.34 70 degree 0.17 80 degree 0.08 85 degree

        // 计算从参考帧（ref_ftr）到当前帧（new_frame_）的变换矩阵 T_cur_ref
        SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();

        // 根据参考帧像素坐标 ref_ftr->px_、3D点坐标 pf、法向量 norm_vec 和帧间变换 T_cur_ref，计算仿射变换矩阵 A_cur_ref_zero
        getWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec, T_cur_ref, 0, A_cur_ref_zero);
        // 根据仿射变换矩阵的行列式值判断需要在图像金字塔中的哪个层级进行搜索
        search_level = getBestSearchLevel(A_cur_ref_zero, 2);
      }
      else
      {
        // 若缓存中已有该参考帧对应的仿射变换信息（通过 warp_map 查找），则直接复用
        auto iter_warp = warp_map.find(ref_ftr->id_);
        if (iter_warp != warp_map.end())
        {
          search_level = iter_warp->second->search_level;
          A_cur_ref_zero = iter_warp->second->A_cur_ref;
        }
        else
        {
          // 如果没有缓存信息，用深度信息、相机模型和帧间变换，计算出一个局部的仿射变换矩阵
          getWarpMatrixAffine(*cam, ref_ftr->px_, ref_ftr->f_, (ref_ftr->pos() - pt->pos_).norm(), new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(),
                              ref_ftr->level_, 0, patch_size_half, A_cur_ref_zero);

          search_level = getBestSearchLevel(A_cur_ref_zero, 2);

          // 保存该放射变换与金字塔层级
          Warp *ot = new Warp(search_level, A_cur_ref_zero);
          warp_map[ref_ftr->id_] = ot;
        }
      }
      // t_4 += omp_get_wtime() - t_1;

      // t_1 = omp_get_wtime();

      for (int pyramid_level = 0; pyramid_level <= patch_pyrimid_level - 1; pyramid_level++)
      {
        // 遍历图像金字塔的不同层级（pyramid_level），对参考帧中的图像块进行变形操作
        // 对图像块进行不同分辨率下的仿射变换
        warpAffine(A_cur_ref_zero, ref_ftr->img_, ref_ftr->px_, ref_ftr->level_, search_level, pyramid_level, patch_size_half, patch_wrap.data());
      }

      // step 10. 双线性插值计算残差
      // 从当前帧图像中提取以 pc 点为中心的图像块，并使用双线性插值方法进行亚像素级别的采样，结果保存在 patch_buffer 中
      getImagePatch(img, pc, patch_buffer.data(), 0);

      float error = 0.0;
      // 遍历图像块中的每一个像素点
      // 计算当前帧图像块（patch_buffer）与参考帧图像块（patch_wrap）之间的像素差值
      for (int ind = 0; ind < patch_size_total; ind++)
      {
        // 曝光时间的倒数，即越亮的图像其值越小
        // patch_wrap 是根据仿射变换矩阵从其他帧“扭曲”过来的图像块
        // patch_buffer 则是当前帧中的原始图像块
        error += (ref_ftr->inv_expo_time_ * patch_wrap[ind] - state->inv_expo_time * patch_buffer[ind]) *
                 (ref_ftr->inv_expo_time_ * patch_wrap[ind] - state->inv_expo_time * patch_buffer[ind]);
      }

      // 计算仿射变换后的图像块与当前帧图像块的归一化互相关
      if (ncc_en)
      {
        double ncc = calculateNCC(patch_wrap.data(), patch_buffer.data(), patch_size_total);
        if (ncc < ncc_thre)
        {
          // grid_num[i] = TYPE_UNKNOWN;
          continue;
        }
      }

      // 跳过误匹配的图像块
      if (error > outlier_threshold * patch_size_total) continue;

      visual_submap->voxel_points.push_back(pt); // 将当前特征点 pt 加入到 visual_submap 的体素点列表中，表示这个点被成功追踪并用于建图
      visual_submap->propa_errors.push_back(error); // 存储当前点的传播误差（propagation error），用于评估追踪质量
      visual_submap->search_levels.push_back(search_level); // 记录在金字塔图像中进行匹配所使用的层级（level），用于后续调整搜索策略
      visual_submap->errors.push_back(error);
      visual_submap->warp_patch.push_back(patch_wrap); // 存储经过仿射变换后对齐的图像块（warped patch），可用于下一帧的追踪或光流计算
      visual_submap->inv_expo_list.push_back(ref_ftr->inv_expo_time_); // 存储参考帧的逆曝光时间（inverse exposure time），用于光度一致性补偿，避免不同光照条件下的误差放大

      // t_5 += omp_get_wtime() - t_1;
    }
  }
  total_points = visual_submap->voxel_points.size();

  // double t3 = omp_get_wtime();
  // cout<<"C. addSubSparseMap: "<<t3-t2<<endl;
  // cout<<"depthcontinuous: C1 "<<t_2<<" C2 "<<t_3<<" C3 "<<t_4<<" C4
  // "<<t_5<<endl;
  printf("[ VIO ] Retrieve %d points from visual sparse map\n", total_points);
}

void VIOManager::computeJacobianAndUpdateEKF(cv::Mat img)
{
  if (total_points == 0) return;
  
  compute_jacobian_time = update_ekf_time = 0.0;
  // patch_pyrimid_level 由文件配置
  for (int level = patch_pyrimid_level - 1; level >= 0; level--)
  {
    // 若启用逆向合成则禁用视觉点参考帧缓存 一般叫 逆光流法，反向光流法 https://blog.csdn.net/guanjing_dream/article/details/129318349
    // TODO: 这里和LIVO1 不同,使用了逆向光流法提高计算效率
    if (inverse_composition_en)
    {
      has_ref_patch_cache = false;
      updateStateInverse(img, level);
    }
    else
    {
      // 在原先的LK光流中，每次迭代都是在当前帧新坐标点附近更新，需要重新计算jacobian和Hessian矩阵
      updateState(img, level);
    }  
  }
  // 利用迭代卡尔曼增益 G 对状态协方差矩阵进行修正，从而减小不确定性
  // P=(I-KH)P
  state->cov -= G * state->cov;
  updateFrameState(*state);
}

void VIOManager::generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg)
{
  if (pg.size() <= 10) return;

  // double t0 = omp_get_wtime();
  // 1. 从点云 pg 中筛选出具有高角点响应值的点，并将其加入到视觉地图中
  for (int i = 0; i < pg.size(); i++)
  {
    if (pg[i].normal == V3D(0, 0, 0)) continue;

    V3D pt = pg[i].point_w;
    V2D pc(new_frame_->w2c(pt)); // lidar 世界坐标点转到当前帧像素坐标

    // 判断该点是否在图像范围内，并且距离图像边缘有一定余量 border 
    if (new_frame_->cam_->isInFrame(pc.cast<int>(), border)) // 20px is the patch size in the matcher
    {
      // 根据像素坐标 pc 计算该点所属的图像网格索引 index
      int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);

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
    }
  }

  // 2. 从视觉子图 visual_submap 中筛选出具有高角点响应值的点，并将其加入到视觉地图中
  // 这些点通常是由光线投射 Raycasting 模块生成的候选特征点
  for (int j = 0; j < visual_submap->add_from_voxel_map.size(); j++)
  {
    // add_from_voxel_map 是一个临时缓存的候选点列表，通常是通过 Raycasting 或其他方式从平面地图中提取的潜在特征点
    V3D pt = visual_submap->add_from_voxel_map[j].point_w;
    V2D pc(new_frame_->w2c(pt));

    if (new_frame_->cam_->isInFrame(pc.cast<int>(), border)) // 20px is the patch size in the matcher
    {
      int index = static_cast<int>(pc[1] / grid_size) * grid_n_width + static_cast<int>(pc[0] / grid_size);

      if (grid_num[index] != TYPE_MAP)
      {
        float cur_value = vk::shiTomasiScore(img, pc[0], pc[1]);
        if (cur_value > scan_value[index])
        {
          scan_value[index] = cur_value;
          append_voxel_points[index] = visual_submap->add_from_voxel_map[j];
          grid_num[index] = TYPE_POINTCLOUD;
        }
      }
    }
  }

  // double t_b1 = omp_get_wtime() - t0;
  // t0 = omp_get_wtime();

  // 3. 从候选点中选取符合几何（点在相机前方）和光照条件的高质量视觉特征点
  int add = 0;
  for (int i = 0; i < length; i++)
  {
    // 表示之前筛选出一个潜在的高质量角点（来自激光雷达或 Raycasting）
    if (grid_num[i] == TYPE_POINTCLOUD) // && (scan_value[i]>=50))
    {
      pointWithVar pt_var = append_voxel_points[i];
      V3D pt = pt_var.point_w;

      // 将候选点转到当前帧相机坐标系下表示
      V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * pt_var.normal);
      V3D dir(new_frame_->T_f_w_ * pt);
      dir.normalize();
      // 方向向量与法向量的夹角余弦
      double cos_theta = dir.dot(norm_vec);
      // if(std::fabs(cos_theta)<0.34) continue; // 70 degree
      V2D pc(new_frame_->w2c(pt));

      float *patch = new float[patch_size_total];
      // 提取以像素坐标 pc 为中心的图像块 patch，采用双线性插值方法处理亚像素精度
      getImagePatch(img, pc, patch, 0);

      VisualPoint *pt_new = new VisualPoint(pt); // 创建新视觉点对象
      Vector3d f = cam->cam2world(pc); // pc 的单位方向向量
      // ftr_new 是该点在当前帧中的观测特征
      Feature *ftr_new = new Feature(pt_new, patch, pc, f, new_frame_->T_f_w_, 0);

      ftr_new->img_ = img;
      ftr_new->id_ = new_frame_->id_;
      ftr_new->inv_expo_time_ = state->inv_expo_time;

      pt_new->addFrameRef(ftr_new);
      pt_new->covariance_ = pt_var.var;
      pt_new->is_normal_initialized_ = true;

      // 是否需要反转法向量方向
      if (cos_theta < 0) { pt_new->normal_ = -pt_var.normal; }
      else { pt_new->normal_ = pt_var.normal; }
      
      pt_new->previous_normal_ = pt_new->normal_;

      // 插入新点到体素地图中
      insertPointIntoVoxelMap(pt_new);
      add += 1;
      // map_cur_frame.push_back(pt_new);
    }
  }

  // double t_b2 = omp_get_wtime() - t0;

  printf("[ VIO ] Append %d new visual map points\n", add);
  // printf("pg.size: %d \n", pg.size());
  // printf("B1. : %.6lf \n", t_b1);
  // printf("B2. : %.6lf \n", t_b2);
}

void VIOManager::updateVisualMapPoints(cv::Mat img)
{
  if (total_points == 0) return;

  int update_num = 0;
  SE3 pose_cur = new_frame_->T_f_w_;
  for (int i = 0; i < total_points; i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];
    if (pt == nullptr) continue;
    // 若该视觉特征点已收敛，则不再更新
    if (pt->is_converged_)
    {
      // 只保留该点的参考观测帧信息 
      pt->deleteNonRefPatchFeatures();
      continue;
    }

    // 提取该点在当前帧中的像素坐标 pc
    V2D pc(new_frame_->w2c(pt->pos_));
    bool add_flag = false; // 是否添加新的观测帧
    
    // 提取该点在当前帧中的图像块 patch_temp
    float *patch_temp = new float[patch_size_total];
    getImagePatch(img, pc, patch_temp, 0);
    // TODO: condition: distance and view_angle
    // Step 1: time

    // 比较该视觉点上一帧观测与当前帧观测的位姿偏移 delta_pose
    Feature *last_feature = pt->obs_.back();
    // if(new_frame_->id_ >= last_feature->id_ + 10) add_flag = true; // 10

    // Step 2: delta_pose
    SE3 pose_ref = last_feature->T_f_w_;
    SE3 delta_pose = pose_ref * pose_cur.inverse();
    // 如果相机移动较大（平移超过 0.5m，旋转超过 0.3 rad），则标记为需更新
    double delta_p = delta_pose.translation().norm();
    double delta_theta = (delta_pose.rotation_matrix().trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (delta_pose.rotation_matrix().trace() - 1));
    if (delta_p > 0.5 || delta_theta > 0.3) add_flag = true; // 0.5 || 0.3

    // Step 3: pixel distance
    // 如果该点在图像上的运动超过一定阈值（如 40 像素）,也需要更新
    Vector2d last_px = last_feature->px_;
    double pixel_dist = (pc - last_px).norm();
    if (pixel_dist > 40) add_flag = true;

    // Maintain the size of 3D point observation features.
    if (pt->obs_.size() >= 30)
    {
      // 如果某个点的观测记录超过最大允许数量（30），就删除得分最低的观测帧
      Feature *ref_ftr;
      pt->findMinScoreFeature(new_frame_->pos(), ref_ftr);
      pt->deleteFeatureRef(ref_ftr);
      // cout<<"pt->obs_.size() exceed 20 !!!!!!"<<endl;
    }
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
  }
  printf("[ VIO ] Update %d points in visual submap\n", update_num);
}

void VIOManager::updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (total_points == 0) return;

  // 遍历当前视觉子地图中的所有特征点
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    if (!pt->is_normal_initialized_) continue;
    if (pt->is_converged_) continue;
    if (pt->obs_.size() <= 5) continue;
    if (update_flag[i] == 0) continue;

    // 计算该视觉地图点的体素位置 loc_xyz[3]
    const V3D &p_w = pt->pos_;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / 0.5;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = plane_map.find(position);
    if (iter != plane_map.end())
    {
      // 查找该点所在的点云平面模型
      VoxelOctoTree *current_octo;
      current_octo = iter->second->find_correspond(p_w);
      if (current_octo->plane_ptr_->is_plane_)
      {
        VoxelPlane &plane = *current_octo->plane_ptr_;
        float dis_to_plane = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_; // 视觉点到点云平面距离
        float dis_to_plane_abs = fabs(dis_to_plane);
        float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                              (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) + (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2)); // 视觉点到点云平面中心的距离平方
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane); // 视觉点到平面中心的投影距离

        // 若位于平面附近 Efficient and Probabilistic Adaptive Voxel Mapping for Accurate online LiDAR Odometry 式11下方
        if (range_dis <= 3 * plane.radius_)
        {
          // Efficient and Probabilistic Adaptive Voxel Mapping for Accurate online LiDAR Odometry 式13
          // 视觉点与平面模型不确定度建模
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
          J_nq.block<1, 3>(0, 3) = -plane.normal_;
          double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
          sigma_l += plane.normal_.transpose() * pt->covariance_ * plane.normal_;

          if (dis_to_plane_abs < 3 * sqrt(sigma_l))
          {
            // V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * plane.normal_);
            // V3D pf(new_frame_->T_f_w_ * pt->pos_);
            // V3D pf_ref(pt->ref_patch->T_f_w_ * pt->pos_);
            // V3D norm_vec_ref(pt->ref_patch->T_f_w_.rotation_matrix() *
            // plane.normal); double cos_ref = pf_ref.dot(norm_vec_ref);
            
            // 检查法向量反向
            if (pt->previous_normal_.dot(plane.normal_) < 0) { pt->normal_ = -plane.normal_; }
            else { pt->normal_ = plane.normal_; }

            double normal_update = (pt->normal_ - pt->previous_normal_).norm();

            pt->previous_normal_ = pt->normal_;

            // 法向量前后变化小且观测帧数量大于10，则认为该点已收敛
            if (normal_update < 0.0001 && pt->obs_.size() > 10)
            {
              pt->is_converged_ = true;
              // visual_converged_point.push_back(pt);
            }
          }
        }
      }
    }

    float score_max = -1000.;
    // 对该视觉点所有的观测帧两两比较
    for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
    {
      Feature *ref_patch_temp = *it;
      float *patch_temp = ref_patch_temp->patch_; // 该视觉点的观测帧1中的图像块
      float NCC_up = 0.0;
      float NCC_down1 = 0.0;
      float NCC_down2 = 0.0;
      float NCC = 0.0;
      float score = 0.0;
      int count = 0;

      V3D pf = ref_patch_temp->T_f_w_ * pt->pos_; // 视觉点在观测帧1下的方向向量
      V3D norm_vec = ref_patch_temp->T_f_w_.rotation_matrix() * pt->normal_; // 视觉点在观测帧1下的法向量
      pf.normalize();
      double cos_angle = pf.dot(norm_vec);
      // if(fabs(cos_angle) < 0.86) continue; // 20 degree

      float ref_mean;
      // 计算该视觉点在观测帧1下图像块区域的灰度均值
      if (abs(ref_patch_temp->mean_) < 1e-6)
      {
        float ref_sum = std::accumulate(patch_temp, patch_temp + patch_size_total, 0.0);
        ref_mean = ref_sum / patch_size_total;
        ref_patch_temp->mean_ = ref_mean;
      }

      for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme; ++itm)
      {
        if ((*itm)->id_ == ref_patch_temp->id_) continue;
        float *patch_cache = (*itm)->patch_;

        float other_mean;
        // 视觉点在观测帧2下图像块区域的灰度均值
        if (abs((*itm)->mean_) < 1e-6)
        {
          float other_sum = std::accumulate(patch_cache, patch_cache + patch_size_total, 0.0);
          other_mean = other_sum / patch_size_total;
          (*itm)->mean_ = other_mean;
        }

        // 计算图像块的NCC
        for (int ind = 0; ind < patch_size_total; ind++)
        {
          NCC_up += (patch_temp[ind] - ref_mean) * (patch_cache[ind] - other_mean);
          NCC_down1 += (patch_temp[ind] - ref_mean) * (patch_temp[ind] - ref_mean);
          NCC_down2 += (patch_cache[ind] - other_mean) * (patch_cache[ind] - other_mean);
        }
        NCC += fabs(NCC_up / sqrt(NCC_down1 * NCC_down2));
        count++;
      }

      NCC = NCC / count;

      // 加上 cos_angle 构造最终评分 score，兼顾光度一致性和几何一致性
      score = NCC + cos_angle;

      // 按照最高评分更新该视觉点的参考图像块
      ref_patch_temp->score_ = score;

      if (score > score_max)
      {
        score_max = score;
        pt->ref_patch = ref_patch_temp;
        pt->has_ref_patch_ = true;
      }
    }

  }
}

void VIOManager::projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (total_points == 0) return;
  // if(new_frame_->id_ != 2) return; //124

  int patch_size = 25; // 图像块大小为25，不是8
  string dir = string(ROOT_DIR) + "Log/ref_cur_combine/";

  cv::Mat result = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat result_normal = cv::Mat::zeros(height, width, CV_8UC1);
  cv::Mat result_dense = cv::Mat::zeros(height, width, CV_8UC1);

  cv::Mat img_photometric_error = new_frame_->img_.clone(); // 用于光度误差计算

  uchar *it = (uchar *)result.data; // 视觉点从参考观测帧仿射变换到当前观测帧的结果
  uchar *it_normal = (uchar *)result_normal.data; // 视觉点从参考观测帧仿射变换到当前观测帧，再反变换回参考观测帧的结果
  uchar *it_dense = (uchar *)result_dense.data;

  struct pixel_member
  {
    Vector2f pixel_pos; // image patch 某个像素的坐标
    uint8_t pixel_value; // image patch 某个像素的值
  };

  int num = 0;
  // 遍历视觉子地图的视觉特征点
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    if (pt->is_normal_initialized_)
    {
      Feature *ref_ftr; // 获取该视觉点对应的参考观测帧信息
      ref_ftr = pt->ref_patch;
      // Feature* ref_ftr;
      V2D pc(new_frame_->w2c(pt->pos_)); // 当前帧下该视觉点的像素
      V2D pc_prior(new_frame_->w2c_prior(pt->pos_)); // 使用先验状态估计的像素坐标

      V3D norm_vec(ref_ftr->T_f_w_.rotation_matrix() * pt->normal_); // 该点的法向量 pt->normal_ 转换到参考观测帧的相机坐标系中
      V3D pf(ref_ftr->T_f_w_ * pt->pos_);

      // 使法向量与方向向量同向
      if (pf.dot(norm_vec) < 0) norm_vec = -norm_vec;

      // norm_vec << norm_vec(1), norm_vec(0), norm_vec(2);
      cv::Mat img_cur = new_frame_->img_; // 当前帧图像
      cv::Mat img_ref = ref_ftr->img_; // 视觉点参考帧图像

      // 参考帧到当前帧变换
      SE3 T_cur_ref = new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse();
      Matrix2d A_cur_ref; // 计算参考帧到当前帧的仿射变换矩阵
      getWarpMatrixAffineHomography(*cam, ref_ftr->px_, pf, norm_vec, T_cur_ref, 0, A_cur_ref);

      // const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
      int search_level = getBestSearchLevel(A_cur_ref.inverse(), 2);

      double D = A_cur_ref.determinant();
      if (D > 3) continue;

      num++;

      cv::Mat ref_cur_combine_temp;
      int radius = 20;
      cv::hconcat(img_cur, img_ref, ref_cur_combine_temp); // 水平拼接当前帧与参考帧图像
      cv::cvtColor(ref_cur_combine_temp, ref_cur_combine_temp, CV_GRAY2BGR);

      // 获取该视觉点在当前帧图像的图像块
      getImagePatch(img_cur, pc, patch_buffer.data(), 0);

      float error_est = 0.0;
      float error_gt = 0.0;

      // 遍历一个图像块中的所有像素
      for (int ind = 0; ind < patch_size_total; ind++)
      {
        // 计算光度一致性误差
        // ref_ftr->inv_expo_time_: 参考帧的逆曝光时间（inverse exposure time），用于补偿光照差异
        // visual_submap->warp_patch[i][ind]: 从参考帧“扭曲”过来的图像块（warped patch）
        // state->inv_expo_time: 当前帧的逆曝光时间
        // patch_buffer[ind]: 当前帧提取的图像块（patch)
        error_est += (ref_ftr->inv_expo_time_ * visual_submap->warp_patch[i][ind] - state->inv_expo_time * patch_buffer[ind]) *
                     (ref_ftr->inv_expo_time_ * visual_submap->warp_patch[i][ind] - state->inv_expo_time * patch_buffer[ind]);
      }
      std::string ref_est = "ref_est " + std::to_string(1.0 / ref_ftr->inv_expo_time_);
      std::string cur_est = "cur_est " + std::to_string(1.0 / state->inv_expo_time);
      std::string cur_propa = "cur_gt " + std::to_string(error_gt);
      std::string cur_optimize = "cur_est " + std::to_string(error_est);

      // plot error result
      cv::putText(ref_cur_combine_temp, ref_est, cv::Point2f(ref_ftr->px_[0] + img_cur.cols - 40, ref_ftr->px_[1] + 40), cv::FONT_HERSHEY_COMPLEX, 0.4,
                  cv::Scalar(0, 255, 0), 1, 8, 0);

      cv::putText(ref_cur_combine_temp, cur_est, cv::Point2f(pc[0] - 40, pc[1] + 40), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, 8, 0);
      cv::putText(ref_cur_combine_temp, cur_propa, cv::Point2f(pc[0] - 40, pc[1] + 60), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255), 1, 8,
                  0);
      cv::putText(ref_cur_combine_temp, cur_optimize, cv::Point2f(pc[0] - 40, pc[1] + 80), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, 8,
                  0);

      cv::rectangle(ref_cur_combine_temp, cv::Point2f(ref_ftr->px_[0] + img_cur.cols - radius, ref_ftr->px_[1] - radius),
                    cv::Point2f(ref_ftr->px_[0] + img_cur.cols + radius, ref_ftr->px_[1] + radius), cv::Scalar(0, 0, 255), 1);
      cv::rectangle(ref_cur_combine_temp, cv::Point2f(pc[0] - radius, pc[1] - radius), cv::Point2f(pc[0] + radius, pc[1] + radius),
                    cv::Scalar(0, 255, 0), 1);
      cv::rectangle(ref_cur_combine_temp, cv::Point2f(pc_prior[0] - radius, pc_prior[1] - radius),
                    cv::Point2f(pc_prior[0] + radius, pc_prior[1] + radius), cv::Scalar(255, 255, 255), 1);
      cv::circle(ref_cur_combine_temp, cv::Point2f(ref_ftr->px_[0] + img_cur.cols, ref_ftr->px_[1]), 1, cv::Scalar(0, 0, 255), -1, 8);
      cv::circle(ref_cur_combine_temp, cv::Point2f(pc[0], pc[1]), 1, cv::Scalar(0, 255, 0), -1, 8);
      cv::circle(ref_cur_combine_temp, cv::Point2f(pc_prior[0], pc_prior[1]), 1, cv::Scalar(255, 255, 255), -1, 8);
      cv::imwrite(dir + std::to_string(new_frame_->id_) + "_" + std::to_string(ref_ftr->id_) + "_" + std::to_string(num) + ".png",
                  ref_cur_combine_temp);

      std::vector<std::vector<pixel_member>> pixel_warp_matrix;

      // 遍历图像块中的像素点
      for (int y = 0; y < patch_size; ++y)
      {
        vector<pixel_member> pixel_warp_vec;
        for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
        {
          Vector2f px_patch(x - patch_size / 2, y - patch_size / 2);
          px_patch *= (1 << search_level);
          const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
          // 在参考帧 vk::interpolateMat_8u 双线性插值获取亚像素值（这里使像素值，不是像素坐标）
          uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);

          // 将px_patch通过仿射变换到当前帧，查找对应像素
          const Vector2f px(A_cur_ref.cast<float>() * px_patch + pc.cast<float>());
          if (px[0] < 0 || px[1] < 0 || px[0] >= img_cur.cols - 1 || px[1] >= img_cur.rows - 1)
            continue;
          else
          {
            pixel_member pixel_warp;
            pixel_warp.pixel_pos << px[0], px[1]; // 该像素点在当前帧下的位置 px
            pixel_warp.pixel_value = pixel_value;
            pixel_warp_vec.push_back(pixel_warp);
          }
        }
        pixel_warp_matrix.push_back(pixel_warp_vec);
      }

      // 从仿射变换后的像素点矩阵中提取整数边界
      float x_min = 1000;
      float y_min = 1000;
      float x_max = 0;
      float y_max = 0;

      for (int i = 0; i < pixel_warp_matrix.size(); i++)
      {
        vector<pixel_member> pixel_warp_row = pixel_warp_matrix[i];
        for (int j = 0; j < pixel_warp_row.size(); j++)
        {
          float x_temp = pixel_warp_row[j].pixel_pos[0];
          float y_temp = pixel_warp_row[j].pixel_pos[1];
          if (x_temp < x_min) x_min = x_temp;
          if (y_temp < y_min) y_min = y_temp;
          if (x_temp > x_max) x_max = x_temp;
          if (y_temp > y_max) y_max = y_temp;
        }
      }
      int x_min_i = floor(x_min);
      int y_min_i = floor(y_min);
      int x_max_i = ceil(x_max);
      int y_max_i = ceil(y_max);

      // 将仿射变换后的像素反投影到参考观测帧中
      Matrix2f A_cur_ref_Inv = A_cur_ref.inverse().cast<float>(); // 视觉点当前观测帧到视觉点参考观测帧
      for (int i = x_min_i; i < x_max_i; i++)
      {
        for (int j = y_min_i; j < y_max_i; j++)
        {
          Eigen::Vector2f pc_temp(i, j);
          Vector2f px_patch = A_cur_ref_Inv * (pc_temp - pc.cast<float>());
          if (px_patch[0] > (-patch_size / 2 * (1 << search_level)) && px_patch[0] < (patch_size / 2 * (1 << search_level)) &&
              px_patch[1] > (-patch_size / 2 * (1 << search_level)) && px_patch[1] < (patch_size / 2 * (1 << search_level)))
          {
            const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
            uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);
            it_normal[width * j + i] = pixel_value;
          }
        }
      }
    }
  }

  // 遍历子地图中的视觉点
  for (int i = 0; i < visual_submap->voxel_points.size(); i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];

    if (!pt->is_normal_initialized_) continue;

    Feature *ref_ftr; // 视觉点pt的参考观测帧
    V2D pc(new_frame_->w2c(pt->pos_)); // 视觉点pt在当前帧下的像素坐标
    ref_ftr = pt->ref_patch;

    Matrix2d A_cur_ref;
    getWarpMatrixAffine(*cam, ref_ftr->px_, ref_ftr->f_, (ref_ftr->pos() - pt->pos_).norm(), new_frame_->T_f_w_ * ref_ftr->T_f_w_.inverse(), 0, 0,
                        patch_size_half, A_cur_ref);
    int search_level = getBestSearchLevel(A_cur_ref.inverse(), 2);
    double D = A_cur_ref.determinant();
    if (D > 3) continue;

    cv::Mat img_cur = new_frame_->img_;
    cv::Mat img_ref = ref_ftr->img_;
    // 像素点从参考观测帧仿射变换到当前观测帧
    for (int y = 0; y < patch_size; ++y)
    {
      for (int x = 0; x < patch_size; ++x) //, ++patch_ptr)
      {
        Vector2f px_patch(x - patch_size / 2, y - patch_size / 2);
        px_patch *= (1 << search_level);
        const Vector2f px_ref(px_patch + ref_ftr->px_.cast<float>());
        uint8_t pixel_value = (uint8_t)vk::interpolateMat_8u(img_ref, px_ref[0], px_ref[1]);

        const Vector2f px(A_cur_ref.cast<float>() * px_patch + pc.cast<float>());
        if (px[0] < 0 || px[1] < 0 || px[0] >= img_cur.cols - 1 || px[1] >= img_cur.rows - 1)
          continue;
        else
        {
          int col = int(px[0]);
          int row = int(px[1]);
          it[width * row + col] = pixel_value;
        }
      }
    }
  }

  cv::Mat ref_cur_combine;
  cv::Mat ref_cur_combine_normal;
  cv::Mat ref_cur_combine_error;

  cv::hconcat(result, new_frame_->img_, ref_cur_combine);
  cv::hconcat(result_normal, new_frame_->img_, ref_cur_combine_normal);

  cv::cvtColor(ref_cur_combine, ref_cur_combine, CV_GRAY2BGR);
  cv::cvtColor(ref_cur_combine_normal, ref_cur_combine_normal, CV_GRAY2BGR);
  cv::absdiff(img_photometric_error, result_normal, img_photometric_error);
  cv::hconcat(img_photometric_error, new_frame_->img_, ref_cur_combine_error);

  cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_" + ".png", ref_cur_combine);
  cv::imwrite(dir + std::to_string(new_frame_->id_) + +"_0_" +
                  "photometric"
                  ".png",
              ref_cur_combine_error);
  cv::imwrite(dir + std::to_string(new_frame_->id_) + "_0_" + "normal" + ".png", ref_cur_combine_normal);
}

void VIOManager::precomputeReferencePatches(int level)
{
  double t1 = omp_get_wtime();
  if (total_points == 0) return;
  MD(1, 2) Jimg; // 图像梯度
  MD(2, 3) Jdpi; // 视觉特征点的像素对相机归一化平面坐标偏导
  MD(1, 3) Jdphi, Jdp;
  MD(1, 3) JdR; // 图像重投影误差jacobian的旋转部分
  MD(1, 3) Jdt; // 图像重投影误差jacobian的平移部分

  // patch_size_total 每个 image patch 包含的像素数
  // total_points 特征点数量
  const int H_DIM = total_points * patch_size_total;

  // 每个像素点对状态向量的偏导
  H_sub_inv.resize(H_DIM, 6);
  H_sub_inv.setZero();
  M3D p_w_hat;

  for (int i = 0; i < total_points; i++)
  {
    const int scale = (1 << level);

    VisualPoint *pt = visual_submap->voxel_points[i]; // 当前视觉特征点
    cv::Mat img = pt->ref_patch->img_; // 当前视觉特征点的参考观测帧

    if (pt == nullptr) continue;

    double depth((pt->pos_ - pt->ref_patch->pos()).norm()); // 视觉点与其参考帧观测点之间的距离
    V3D pf = pt->ref_patch->f_ * depth; // 视觉点在参考观测帧下的方向向量
    V2D pc = pt->ref_patch->px_; // 视觉点在参考观测帧下的像素
    M3D R_ref_w = pt->ref_patch->T_f_w_.rotation_matrix(); // 参考观测帧的旋转矩阵

    // 重投影残差对预测的视觉特征点相机坐标求偏导
    computeProjectionJacobian(pf, Jdpi);
    p_w_hat << SKEW_SYM_MATRX(pt->pos_);

    // 基于双线性插值的亚像素计算
    const float u_ref = pc[0];
    const float v_ref = pc[1];
    const int u_ref_i = floorf(pc[0] / scale) * scale;
    const int v_ref_i = floorf(pc[1] / scale) * scale;
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;
    const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref); // 左上角权重
    const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref); // 右上角权重
    const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref; // 左下角权重
    const float w_ref_br = subpix_u_ref * subpix_v_ref; // 右下角权重

    // 遍历图像块中的每个像素
    for (int x = 0; x < patch_size; x++)
    {
      uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
      // 计算每个像素的梯度 du dv
      for (int y = 0; y < patch_size; ++y, img_ptr += scale)
      {
        float du =
            0.5f *
            ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] + w_ref_bl * img_ptr[scale * width + scale] +
              w_ref_br * img_ptr[scale * width + scale * 2]) -
             (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] + w_ref_bl * img_ptr[scale * width - scale] + w_ref_br * img_ptr[scale * width]));
        float dv =
            0.5f *
            ((w_ref_tl * img_ptr[scale * width] + w_ref_tr * img_ptr[scale + scale * width] + w_ref_bl * img_ptr[width * scale * 2] +
              w_ref_br * img_ptr[width * scale * 2 + scale]) -
             (w_ref_tl * img_ptr[-scale * width] + w_ref_tr * img_ptr[-scale * width + scale] + w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));

        // 归一化像素梯度，适应金字塔层级
        Jimg << du, dv;
        Jimg = Jimg * (1.0 / scale);

        // 重投影误差模型的雅克比矩阵
        // 右扰动模型
        JdR = Jimg * Jdpi * R_ref_w * p_w_hat;
        Jdt = -Jimg * Jdpi * R_ref_w;

        // 存储每个像素对应的 Jacobian
        H_sub_inv.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt;
      }
    }
  }
  has_ref_patch_cache = true;
}

void VIOManager::updateStateInverse(cv::Mat img, int level)
{
  if (total_points == 0) return;
  StatesGroup old_state = (*state);
  V2D pc;
  MD(1, 2) Jimg;
  MD(2, 3) Jdpi;
  MD(1, 3) Jdphi, Jdp, JdR, Jdt;
  VectorXd z; // 保存残差
  MatrixXd H_sub; // 重投影jacobian映射到当前imu状态空间
  bool EKF_end = false;
  float last_error = std::numeric_limits<float>::max();
  compute_jacobian_time = update_ekf_time = 0.0;
  M3D P_wi_hat;
  bool z_init = true;
  const int H_DIM = total_points * patch_size_total;

  z.resize(H_DIM);
  z.setZero();

  H_sub.resize(H_DIM, 6);
  H_sub.setZero();

  // 进IEKF迭代更新状态量
  for (int iteration = 0; iteration < max_iterations; iteration++)
  {
    double t1 = omp_get_wtime();
    double count_outlier = 0;

    // 这里参考视觉帧像素的jacobian只计算了一遍，对比updateState()节约了计算量
    if (has_ref_patch_cache == false) 
    {
      precomputeReferencePatches(level); // has_ref_patch_cache 在precomputeReferencePatches()内置为true
    }
    int n_meas = 0;
    float error = 0.0;

    // 使用IEKF等价代替GN迭代位姿变换增量
    M3D Rwi(state->rot_end); // 当前帧imu的状态估计旋转
    V3D Pwi(state->pos_end); // 当前帧imu的状态估计平移
    P_wi_hat << SKEW_SYM_MATRX(Pwi);
    Rcw = Rci * Rwi.transpose(); // 当前帧相机到世界坐标系的旋转
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci; // 当前帧相机到世界坐标系的平移
    M3D p_hat;

    // 对所有视觉点，在卡尔曼滤波中构建观测残差和对应的雅可比矩阵（Jacobian）
    for (int i = 0; i < total_points; i++)
    {
      float patch_error = 0.0;

      const int scale = (1 << level);

      VisualPoint *pt = visual_submap->voxel_points[i];

      if (pt == nullptr) continue;

      V3D pf = Rcw * pt->pos_ + Pcw; // 视觉特征点从世界坐标转到当前帧相机坐标系下
      pc = cam->world2cam(pf);

      const float u_ref = pc[0];
      const float v_ref = pc[1];
      const int u_ref_i = floorf(pc[0] / scale) * scale;
      const int v_ref_i = floorf(pc[1] / scale) * scale;
      const float subpix_u_ref = (u_ref - u_ref_i) / scale;
      const float subpix_v_ref = (v_ref - v_ref_i) / scale;
      const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
      const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
      const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
      const float w_ref_br = subpix_u_ref * subpix_v_ref;

      vector<float> P = visual_submap->warp_patch[i]; // warp_patch[i]: 之前通过仿射变换对齐的图像块数据
      for (int x = 0; x < patch_size; x++)
      {
        uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale)
        {
          // 实际像素值-仿射变换预测像素值，作为残差
          double res = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] +
                       w_ref_br * img_ptr[scale * width + scale] - P[patch_size_total * level + x * patch_size + y];
          z(i * patch_size_total + x * patch_size + y) = res;
          patch_error += res * res;
          MD(1, 3) J_dR = H_sub_inv.block<1, 3>(i * patch_size_total + x * patch_size + y, 0);
          MD(1, 3) J_dt = H_sub_inv.block<1, 3>(i * patch_size_total + x * patch_size + y, 3);

          // 将残差从图像梯度映射到当前imu状态空间
          JdR = J_dR * Rwi + J_dt * P_wi_hat * Rwi;
          Jdt = J_dt * Rwi;
          H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt;
          n_meas++;
        }
      }
      visual_submap->errors[i] = patch_error; // 记录图像块残差平方
      error += patch_error;
    }

    error = error / n_meas; // 所有特征点误差总和，用于迭代收敛判断

    compute_jacobian_time += omp_get_wtime() - t1;

    double t3 = omp_get_wtime();

    // 如果当前误差小于或等于上次误差，则继续更新
    if (error <= last_error)
    {
      old_state = (*state);
      last_error = error;

      // 以下是IEKF更新公式，具体可见 voxel_map.cpp StateEstimation()
      auto &&H_sub_T = H_sub.transpose();
      H_T_H.setZero();
      G.setZero();
      H_T_H.block<6, 6>(0, 0) = H_sub_T * H_sub;
      // 结合
      MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
      auto &&HTz = H_sub_T * z;
      // vec 为 (x_predict ⊟ x_k) 先验预测值-IEKF第k次迭代值
      auto vec = (*state_propagat) - (*state);
      G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
      
      // 状态增量
      auto solution = -K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);
      (*state) += solution;
      auto &&rot_add = solution.block<3, 1>(0, 0);
      auto &&t_add = solution.block<3, 1>(3, 0);

      if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f)) { EKF_end = true; }
    }
    else
    {
      // 否则回滚状态
      (*state) = old_state;
      EKF_end = true;
    }

    update_ekf_time += omp_get_wtime() - t3;

    if (iteration == max_iterations || EKF_end) break; 
  }
}

void VIOManager::updateState(cv::Mat img, int level)
{
  if (total_points == 0) return;
  StatesGroup old_state = (*state);

  VectorXd z;
  MatrixXd H_sub;
  bool EKF_end = false;
  float last_error = std::numeric_limits<float>::max();

  const int H_DIM = total_points * patch_size_total;
  z.resize(H_DIM);
  z.setZero();
  H_sub.resize(H_DIM, 7);
  H_sub.setZero();

  for (int iteration = 0; iteration < max_iterations; iteration++)
  {
    double t1 = omp_get_wtime();

    // 使用IEKF等价代替GN迭代位姿变换增量
    M3D Rwi(state->rot_end); // 当前帧imu的状态估计旋转
    V3D Pwi(state->pos_end); // 当前帧imu的状态估计平移
    Rcw = Rci * Rwi.transpose(); // 当前帧相机到世界坐标系的旋转
    Pcw = -Rci * Rwi.transpose() * Pwi + Pci; // 当前帧相机到世界坐标系的平移
    Jdp_dt = Rci * Rwi.transpose(); // Pcw 对 Pwi 求偏导，将 IMU 平移误差映射到相机坐标空间
    
    float error = 0.0;
    int n_meas = 0;
    // int max_threads = omp_get_max_threads();
    // int desired_threads = std::min(max_threads, total_points);
    // omp_set_num_threads(desired_threads);
  
    #ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
      #pragma omp parallel for reduction(+:error, n_meas) // reduction 在每个线程里面对变量 data 进行拷贝，然后在线程当中使用这个拷贝的变量，避免数据竞争
    #endif
    for (int i = 0; i < total_points; i++)
    {
      // 为每个候选点计算 jacobian 
      // printf("thread is %d, i=%d, i address is %p\n", omp_get_thread_num(), i, &i);
      MD(1, 2) Jimg;
      MD(2, 3) Jdpi;
      MD(1, 3) Jdphi, Jdp, JdR, Jdt;

      float patch_error = 0.0; // 当前图像块所有像素光度误差平方和
      int search_level = visual_submap->search_levels[i]; // 表示第 i 个特征点在图像金字塔中选择的搜索层级
      int pyramid_level = level + search_level; // level 当前函数调用的图像金字塔层级（如从粗到精），pyramid_level: 最终要提取图像块的金字塔层级
      int scale = (1 << pyramid_level); // 图像缩放比例
      float inv_scale = 1.0f / scale; // 用于归一化图像梯度（gradient normalization），避免不同层级下梯度大小不一致导致 Jacobian 失真

      VisualPoint *pt = visual_submap->voxel_points[i];

      if (pt == nullptr) continue;

      // 视觉点转到相机坐标系
      V3D pf = Rcw * pt->pos_ + Pcw;
      V2D pc = cam->world2cam(pf);

      computeProjectionJacobian(pf, Jdpi);
      M3D p_hat;
      p_hat << SKEW_SYM_MATRX(pf);

      float u_ref = pc[0];
      float v_ref = pc[1];
      int u_ref_i = floorf(pc[0] / scale) * scale;
      int v_ref_i = floorf(pc[1] / scale) * scale;
      float subpix_u_ref = (u_ref - u_ref_i) / scale;
      float subpix_v_ref = (v_ref - v_ref_i) / scale;
      float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
      float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
      float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
      float w_ref_br = subpix_u_ref * subpix_v_ref;

      // warp_patch[i]: 之前通过仿射变换对齐的图像块数据
      vector<float> P = visual_submap->warp_patch[i];
      // inv_expo_list[i]: 存储了参考帧的逆曝光时间，用于补偿光照变化
      double inv_ref_expo = visual_submap->inv_expo_list[i];
      // ROS_ERROR("inv_ref_expo: %.3lf, state->inv_expo_time: %.3lf\n", inv_ref_expo, state->inv_expo_time);

      for (int x = 0; x < patch_size; x++)
      {
        uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i + x * scale - patch_size_half * scale) * width + u_ref_i - patch_size_half * scale;
        for (int y = 0; y < patch_size; ++y, img_ptr += scale)
        {
          // 像素x方向梯度du
          float du =
              0.5f *
              ((w_ref_tl * img_ptr[scale] + w_ref_tr * img_ptr[scale * 2] + w_ref_bl * img_ptr[scale * width + scale] +
                w_ref_br * img_ptr[scale * width + scale * 2]) -
               (w_ref_tl * img_ptr[-scale] + w_ref_tr * img_ptr[0] + w_ref_bl * img_ptr[scale * width - scale] + w_ref_br * img_ptr[scale * width]));

          // 像素y方向梯度dv
          float dv =
              0.5f *
              ((w_ref_tl * img_ptr[scale * width] + w_ref_tr * img_ptr[scale + scale * width] + w_ref_bl * img_ptr[width * scale * 2] +
                w_ref_br * img_ptr[width * scale * 2 + scale]) -
               (w_ref_tl * img_ptr[-scale * width] + w_ref_tr * img_ptr[-scale * width + scale] + w_ref_bl * img_ptr[0] + w_ref_br * img_ptr[scale]));

          // 计算归一化图像梯度
          Jimg << du, dv;
          Jimg = Jimg * state->inv_expo_time;
          Jimg = Jimg * inv_scale;

          Jdphi = Jimg * Jdpi * p_hat; // 投影误差对相机归一化坐标的偏导
          Jdp = -Jimg * Jdpi;

          // TODO: 观测误差对 IMU 状态的偏导，从投影空间映射到imu状态空间
          JdR = Jdphi * Jdphi_dR + Jdp * Jdp_dR;
          Jdt = Jdp * Jdp_dt;

          // 当前像素的加权灰度值
          double cur_value =
              w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * width] + w_ref_br * img_ptr[scale * width + scale];
          
          // 构建残差, 乘以逆曝光时间平衡亮度变化
          double res = state->inv_expo_time * cur_value - inv_ref_expo * P[patch_size_total * level + x * patch_size + y];

          z(i * patch_size_total + x * patch_size + y) = res;

          patch_error += res * res;
          n_meas += 1;
          
          // 保存 jacobian 如果启用了曝光估计，则 Jacobian 维度为 7
          if (exposure_estimate_en) { H_sub.block<1, 7>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt, cur_value; }
          else { H_sub.block<1, 6>(i * patch_size_total + x * patch_size + y, 0) << JdR, Jdt; }
        }
      }
      visual_submap->errors[i] = patch_error;
      error += patch_error;
    }

    error = error / n_meas;
    
    compute_jacobian_time += omp_get_wtime() - t1;

    // printf("\nPYRAMID LEVEL %i\n---------------\n", level);
    // std::cout << "It. " << iteration
    //           << "\t last_error = " << last_error
    //           << "\t new_error = " << error
    //           << std::endl;

    double t3 = omp_get_wtime();

    if (error <= last_error)
    {
      old_state = (*state);
      last_error = error;

      // K = (H.transpose() / img_point_cov * H + state->cov.inverse()).inverse() * H.transpose() / img_point_cov; auto
      // vec = (*state_propagat) - (*state); G = K*H;
      // (*state) += (-K*z + vec - G*vec);

      // IEKF 计算状态增量
      // TODO: IEKF与gauss-newtow法的等效性
      auto &&H_sub_T = H_sub.transpose();
      H_T_H.setZero();
      G.setZero();
      H_T_H.block<7, 7>(0, 0) = H_sub_T * H_sub;
      MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H + (state->cov / img_point_cov).inverse()).inverse();
      auto &&HTz = H_sub_T * z;
      // K = K_1.block<DIM_STATE,6>(0,0) * H_sub_T;
      auto vec = (*state_propagat) - (*state); // x_predic - x_k
      G.block<DIM_STATE, 7>(0, 0) = K_1.block<DIM_STATE, 7>(0, 0) * H_T_H.block<7, 7>(0, 0);
      MD(DIM_STATE, 1)
      solution = -K_1.block<DIM_STATE, 7>(0, 0) * HTz + vec - G.block<DIM_STATE, 7>(0, 0) * vec.block<7, 1>(0, 0);

      (*state) += solution;
      auto &&rot_add = solution.block<3, 1>(0, 0);
      auto &&t_add = solution.block<3, 1>(3, 0);

      auto &&expo_add = solution.block<1, 1>(6, 0);
      // if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f) && (expo_add.norm() < 0.001f)) EKF_end = true;
      if ((rot_add.norm() * 57.3f < 0.001f) && (t_add.norm() * 100.0f < 0.001f))  EKF_end = true;
    }
    else
    {
      (*state) = old_state;
      EKF_end = true;
    }

    update_ekf_time += omp_get_wtime() - t3;

    if (iteration == max_iterations || EKF_end) break;
  }
  // if (state->inv_expo_time < 0.0)  {ROS_ERROR("reset expo time!!!!!!!!!!\n"); state->inv_expo_time = 0.0;}
}

void VIOManager::updateFrameState(StatesGroup state)
{
  M3D Rwi(state.rot_end);
  V3D Pwi(state.pos_end);
  Rcw = Rci * Rwi.transpose();
  Pcw = -Rci * Rwi.transpose() * Pwi + Pci;
  new_frame_->T_f_w_ = SE3(Rcw, Pcw);// 当前帧相机位姿
}

void VIOManager::plotTrackedPoints()
{
  int total_points = visual_submap->voxel_points.size(); // 视觉子地图，视觉特征点数量
  if (total_points == 0) return;
  // int inlier_count = 0;
  // for (int i = 0; i < img_cp.rows / grid_size; i++)
  // {
  //   cv::line(img_cp, cv::Poaint2f(0, grid_size * i), cv::Point2f(img_cp.cols, grid_size * i), cv::Scalar(255, 255, 255), 1, CV_AA);
  // }
  // for (int i = 0; i < img_cp.cols / grid_size; i++)
  // {
  //   cv::line(img_cp, cv::Point2f(grid_size * i, 0), cv::Point2f(grid_size * i, img_cp.rows), cv::Scalar(255, 255, 255), 1, CV_AA);
  // }
  // for (int i = 0; i < img_cp.rows / grid_size; i++)
  // {
  //   cv::line(img_cp, cv::Point2f(0, grid_size * i), cv::Point2f(img_cp.cols, grid_size * i), cv::Scalar(255, 255, 255), 1, CV_AA);
  // }
  // for (int i = 0; i < img_cp.cols / grid_size; i++)
  // {
  //   cv::line(img_cp, cv::Point2f(grid_size * i, 0), cv::Point2f(grid_size * i, img_cp.rows), cv::Scalar(255, 255, 255), 1, CV_AA);
  // }
  for (int i = 0; i < total_points; i++)
  {
    VisualPoint *pt = visual_submap->voxel_points[i];
    V2D pc(new_frame_->w2c(pt->pos_));

    if (visual_submap->errors[i] <= visual_submap->propa_errors[i])
    {
      // inlier_count++;
      cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(0, 255, 0), -1, 8); // Green Sparse Align tracked
    }
    else
    {
      cv::circle(img_cp, cv::Point2f(pc[0], pc[1]), 7, cv::Scalar(255, 0, 0), -1, 8); // Blue Sparse Align tracked
    }
  }
  // std::string text = std::to_string(inlier_count) + " " + std::to_string(total_points);
  // cv::Point2f origin;
  // origin.x = img_cp.cols - 110;
  // origin.y = 20;
  // cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, 8, 0);
}

V3F VIOManager::getInterpolatedPixel(cv::Mat img, V2D pc)
{
  const float u_ref = pc[0];
  const float v_ref = pc[1];
  const int u_ref_i = floorf(pc[0]);
  const int v_ref_i = floorf(pc[1]);

  // 当前点距离左上角像素的横向和纵向偏移（范围在 [0,1) 之间）
  const float subpix_u_ref = (u_ref - u_ref_i);
  const float subpix_v_ref = (v_ref - v_ref_i);

  // 计算周围四个像素对该点颜色的权重
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;


  // (uint8_t *)img.data 图像的原始数据指针
  // 每个像素有 3 个字节（RGB），所以乘以 3
  // 得到的是左上角像素的数据地址
  uint8_t *img_ptr = (uint8_t *)img.data + ((v_ref_i)*width + (u_ref_i)) * 3;

  float B = w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[0 + 3] + w_ref_bl * img_ptr[width * 3] + w_ref_br * img_ptr[width * 3 + 0 + 3];
  float G = w_ref_tl * img_ptr[1] + w_ref_tr * img_ptr[1 + 3] + w_ref_bl * img_ptr[1 + width * 3] + w_ref_br * img_ptr[width * 3 + 1 + 3];
  float R = w_ref_tl * img_ptr[2] + w_ref_tr * img_ptr[2 + 3] + w_ref_bl * img_ptr[2 + width * 3] + w_ref_br * img_ptr[width * 3 + 2 + 3];
  V3F pixel(B, G, R);
  return pixel;
}

void VIOManager::dumpDataForColmap()
{
  static int cnt = 1;
  std::ostringstream ss;
  ss << std::setw(5) << std::setfill('0') << cnt; // 固定长度为5的字符串，前面补0
  std::string cnt_str = ss.str();
  std::string image_path = std::string(ROOT_DIR) + "Log/Colmap/images/" + cnt_str + ".png";
  
  cv::Mat img_rgb_undistort;
  pinhole_cam->undistortImage(img_rgb, img_rgb_undistort); // 对当前帧图像去畸变
  cv::imwrite(image_path, img_rgb_undistort);
  
  // 获取当前帧的相机位姿
  Eigen::Quaterniond q(new_frame_->T_f_w_.rotation_matrix());
  Eigen::Vector3d t = new_frame_->T_f_w_.translation();
  fout_colmap << cnt << " "
            << std::fixed << std::setprecision(6)  // 保证浮点数精度为6位
            << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << 1 << " "  // CAMERA_ID (假设相机ID为1)
            << cnt_str << ".png" << std::endl;
  fout_colmap << "0.0 0.0 -1" << std::endl;
  // IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
  // example: 1 0.707107 0.000000 0.000000 0.707107 0.1 0.2 0.3 1 00001.png
  cnt++;
}

void VIOManager::processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time)
{
  if (width != img.cols || height != img.rows)
  {
    if (img.empty()) printf("[ VIO ] Empty Image!\n");
    cv::resize(img, img, cv::Size(img.cols * image_resize_factor, img.rows * image_resize_factor), 0, 0, CV_INTER_LINEAR);
  }
  img_rgb = img.clone();
  img_cp = img.clone();
  // img_test = img.clone();

  // 转灰度图
  if (img.channels() == 3) cv::cvtColor(img, img, CV_BGR2GRAY);

  new_frame_.reset(new Frame(cam, img)); // 初始化当前帧图像

  // 1. 基于LIO先验更新当前图像帧状态
  updateFrameState(*state);
  
  // 重置图像网格信息
  resetGrid();

  double t1 = omp_get_wtime();

  // 2. 生成视觉地图
  retrieveFromVisualSparseMap(img, pg, feat_map);

  double t2 = omp_get_wtime();

  // 3. 基于LK光流和IEKF更新当前图像帧状态
  computeJacobianAndUpdateEKF(img);

  double t3 = omp_get_wtime();

  // 4. 对没有地图点占据的网格，依次在lidar点，光线投射采样点（保存在visual_submap->add_from_voxel_map）点集中选择高分的shiTomasi点，插入到 feat_map 体素地图中
  generateVisualMapPoints(img, pg);

  double t4 = omp_get_wtime();
  
  // 5. 绘制视觉子地图中的视觉特征点
  plotTrackedPoints();

  if (plot_flag) projectPatchFromRefToCur(feat_map);

  double t5 = omp_get_wtime();

  // 6. 为视觉特征添加新的图像帧信息
  updateVisualMapPoints(img);

  double t6 = omp_get_wtime();

  // 7. 检查视觉点是否收敛，并更新视觉点的参考图像帧
  updateReferencePatch(feat_map);

  double t7 = omp_get_wtime();
  
  // 结果保存为colmap格式
  if(colmap_output_en)  dumpDataForColmap();

  frame_count++;
  ave_total = ave_total * (frame_count - 1) / frame_count + (t7 - t1 - (t5 - t4)) / frame_count;

  // printf("[ VIO ] feat_map.size(): %zu\n", feat_map.size());
  // printf("\033[1;32m[ VIO time ]: current frame: retrieveFromVisualSparseMap time: %.6lf secs.\033[0m\n", t2 - t1);
  // printf("\033[1;32m[ VIO time ]: current frame: computeJacobianAndUpdateEKF time: %.6lf secs, comp H: %.6lf secs, ekf: %.6lf secs.\033[0m\n", t3 - t2, computeH, ekf_time);
  // printf("\033[1;32m[ VIO time ]: current frame: generateVisualMapPoints time: %.6lf secs.\033[0m\n", t4 - t3);
  // printf("\033[1;32m[ VIO time ]: current frame: updateVisualMapPoints time: %.6lf secs.\033[0m\n", t6 - t5);
  // printf("\033[1;32m[ VIO time ]: current frame: updateReferencePatch time: %.6lf secs.\033[0m\n", t7 - t6);
  // printf("\033[1;32m[ VIO time ]: current total time: %.6lf, average total time: %.6lf secs.\033[0m\n", t7 - t1 - (t5 - t4), ave_total);

  // ave_build_residual_time = ave_build_residual_time * (frame_count - 1) / frame_count + (t2 - t1) / frame_count;
  // ave_ekf_time = ave_ekf_time * (frame_count - 1) / frame_count + (t3 - t2) / frame_count;
 
  // cout << BLUE << "ave_build_residual_time: " << ave_build_residual_time << RESET << endl;
  // cout << BLUE << "ave_ekf_time: " << ave_ekf_time << RESET << endl;
  
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         VIO Time                            |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27zu |\033[0m\n", "Sparse Map Size", feat_map.size());
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "retrieveFromVisualSparseMap", t2 - t1);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "computeJacobianAndUpdateEKF", t3 - t2);
  printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> computeJacobian", compute_jacobian_time);
  printf("\033[1;32m| %-27s   | %-27lf |\033[0m\n", "-> updateEKF", update_ekf_time);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "generateVisualMapPoints", t4 - t3);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "updateVisualMapPoints", t6 - t5);
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "updateReferencePatch", t7 - t6);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Current Total Time", t7 - t1 - (t5 - t4));
  printf("\033[1;32m| %-29s | %-27lf |\033[0m\n", "Average Total Time", ave_total);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  // std::string text = std::to_string(int(1 / (t7 - t1 - (t5 - t4)))) + " HZ";
  // cv::Point2f origin;
  // origin.x = 20;
  // origin.y = 20;
  // cv::putText(img_cp, text, origin, cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, 8, 0);
  // cv::imwrite("/home/chunran/Desktop/raycasting/" + std::to_string(new_frame_->id_) + ".png", img_cp);
}