# ROS体积估计工作流使用说明
## 初始化工作流

新终端
source ./catkin_ws/devel/setup.bash

roslaunch ros_volume_estimation volume_estimation_pipeline_service.launch

## 发布点云
新终端
rosrun pcl_ros pcd_to_pointcloud \
./fusion_local_downsampled.pcd \
0.1 _frame_id:=/base_link cloud_pcd:=/pointcloud_raw

## 启动工作流
新终端
source ./catkin_ws/devel/setup.bash
rosservice call /volume_estimation/toggle_processing "enabled: true"


## 暂停工作流
新终端
source ./catkin_ws/devel/setup.bash
rosservice call /volume_estimation/toggle_processing "enabled: false"

## 查看估算体积
新终端
rostopic echo /volume_result




# pcd tool使用说明

## 快捷键一览

- **L**: 通过文件对话框加载点云（支持 .npy/.pcd/.ply/.obj/.vtp/.vtk/.stl 等）
- **B**: 创建/重置可交互的红色盒子（Box）；UI 左下角实时显示“Box内最低点 z”
- **O**: 导出当前红盒的 8 个角点为 `obb_corners.txt`，并以黄色线框显示
- **T**: 基于盒内点拟合水平面并将第一主轴对齐到 +X（含 yaw），仅导出 4x4 变换到 `T_level_yaw.txt`（不应用到点云）
- **Space**: 与 T 相同，但会立即将该变换应用到当前点云，同时保存 `T_level_yaw.txt`
- **R**: 导出并应用“Space 变换”的 Z 轴翻转版本（世界坐标 Z 取反），并保存为 `T_level_yaw.txt`（Z-Flip）
- **F**: 选择外部 4x4 矩阵文本（.txt/.csv/.dat），并将该变换应用到当前点云
- **V**: 进行 DEM 体积估计（将盒底面法向对齐至世界 +Z，并以盒内最低点为 z=0）
  - 默认参数：`grid_res_x=0.1`、`grid_res_y=0.1`、`height_method='mean'`、`interpolate_empty_cells=True`、`max_interpolation_passes=100`
  - 控制台输出总体积和插值统计信息

### 盒子移动/旋转
- **A / D**: 沿 X 轴负/正方向平移盒子（步长 0.1）
- **W / X**: 沿 Y 轴正/负方向平移盒子（步长 0.1）
- **Z / C**: 沿 Z 轴上/下移动盒子（步长 0.001）
- **Left / Right**: 绕世界 Z 轴、以盒子中心为枢轴旋转盒子（每次 ±3°）
