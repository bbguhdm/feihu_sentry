
cd /home/feihu_sentry/shaobing/feihu_sentry
source install/setup.bash
source /opt/ros/humble/setup.bash


# 命令数组  . install/setup.bash
commands=(
"gnome-terminal -- bash -c ' ros2 launch robot_description bring.launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch linefit_ground_segmentation_ros segmentation.launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch imu_complementary_filter complementary_filter.launch.py  ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch realsense2_camera rs_align_depth_launch.py  ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch livox_ros_driver2 msg_MID360_launch.py   ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch rtab_localization rtab_localization_launch.py   ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch fast_lio mapping.launch.py    ; exec bash;'"
# "sleep 1
"gnome-terminal -- bash -c ' ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch robot_navigation2 bring_launch.py  rviz_config:=rtab_nav ; exec bash;'"
)

cd /home/feihu_sentry/shaobing/feihu_sentry
for command in "${commands[@]}"; do
  
  eval "$command"
done









                                                                                                                  





