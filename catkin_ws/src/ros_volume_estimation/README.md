# ROS Volume Estimation Package

煤堆体积估算ROS包，支持传统启动模式和服务控制模式。

## 编译和运行

### 1. 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 运行方式

#### 2.1 服务控制模式（推荐）
```bash
# 启动服务控制节点
roslaunch ros_volume_estimation volume_estimation_pipeline_service.launch

# 在另一个终端启动处理流程
rosservice call /volume_estimation/toggle_processing "enabled: true"

# 发布点云数据
rosrun pcl_ros pcd_to_pointcloud ./at128_seg.pcd 0.1 _frame_id:=/base_link cloud_pcd:=/pointcloud_raw

# 停止处理流程
rosservice call /volume_estimation/toggle_processing "enabled: false"

# 查看处理状态
rostopic echo /volume_estimation/status -n 1
```

#### 2.2 传统启动模式
```bash
# 启动所有节点
roslaunch ros_volume_estimation volume_estimation_pipeline.launch

# 在另一个终端发布点云数据
rostopic pub /pointcloud_raw sensor_msgs/PointCloud2 [点云数据]
```

### 3. 测试单个节点
```bash

# 测试变换节点
roslaunch ros_volume_estimation test_single_node.launch node:=transform

# 测试裁剪节点
roslaunch ros_volume_estimation test_single_node.launch node:=crop_filter

# 测试体积估算节点
roslaunch ros_volume_estimation test_single_node.launch node:=volume_estimator
```

## 从PCD文件发布点云

可以使用pcl_ros包从PCD文件发布点云：

```bash
# 安装pcl_ros工具
sudo apt-get install ros-melodic-pcl-ros

# 从PCD文件发布点云
rosrun pcl_ros pcd_to_pointcloud [pcd文件路径] 0.1 _frame_id:=/base_link cloud_pcd:=/pointcloud_raw
```

## 监控Topic和服务

### Topic监控
```bash
# 查看所有topic
rostopic list

# 监控点云数据
rostopic echo /pointcloud_raw
rostopic echo /pointcloud_transformed
rostopic echo /pointcloud_cropped

# 监控体积结果
rostopic echo /volume_result

# 监控处理状态（服务控制模式）
rostopic echo /volume_estimation/status
```

### 服务控制
```bash
# 查看可用服务
rosservice list

# 启动处理流程
rosservice call /volume_estimation/toggle_processing "enabled: true"

# 停止处理流程
rosservice call /volume_estimation/toggle_processing "enabled: false"

# 查看服务信息
rosservice info /volume_estimation/toggle_processing
```



## 参数调整

可以通过修改launch文件中的参数来调整处理效果：


- **变换矩阵**: 根据实际标定结果修改`transformation_matrix`
- **裁剪区域**: 修改`polygons`参数来定义感兴趣区域
- **网格分辨率**: 调整`grid_resolution_x/y`来控制体积估算精度

## 功能特性

### 服务控制模式新增功能
- **动态启停**: 通过ROS服务控制工作流的启动和停止
- **状态反馈**: 实时查看处理状态（RUNNING/STOPPED）
- **进程管理**: 自动管理子进程的创建和销毁
- **错误处理**: 完整的错误处理和恢复机制

### 数据流程
```
 
/pointcloud_raw → transform → /pointcloud_transformed 
                → crop_filter → /pointcloud_cropped 
                → volume_estimator → /volume_result
```

## 依赖包

- roscpp
- sensor_msgs
- std_msgs
- pcl_ros
- pcl_conversions
- message_generation
- message_runtime
- PCL (Point Cloud Library)
- Eigen3

## 文件结构

```
ros_volume_estimation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   ├── transform_node.cpp
│   ├── crop_filter_node.cpp
│   ├── volume_estimator_node.cpp
│   └── workflow_control_node.cpp        # 新增服务控制节点
├── launch/
│   ├── volume_estimation_pipeline.launch
│   ├── volume_estimation_pipeline_service.launch  # 新增服务控制启动文件
│   ├── test_single_node.launch
│   └── workflow_control.launch          # 新增工作流控制启动文件
└── srv/
    └── ToggleProcessing.srv             # 新增服务定义
```

