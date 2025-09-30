# 初始化工作流

新终端
source ./catkin_ws/devel/setup.bash

roslaunch ros_volume_estimation volume_estimation_pipeline_service.launch

# 发布点云
新终端
rosrun pcl_ros pcd_to_pointcloud \
./test_1.pcd \
0.1 _frame_id:=/base_link cloud_pcd:=/pointcloud_raw

# 启动工作流
新终端
source ./catkin_ws/devel/setup.bash
rosservice call /volume_estimation/toggle_processing "enabled: true"


# 暂停工作流
新终端
source ./catkin_ws/devel/setup.bash
rosservice call /volume_estimation/toggle_processing "enabled: false"

# 查看估算体积
新终端
rostopic echo /volume_result
