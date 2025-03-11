
cd /home/feihu_sentry/shaobing/feihu_sentry
source install/setup.bash
source /opt/ros/humble/setup.bash


# 命令数组  . install/setup.bash
commands=(
"gnome-terminal -- bash -c ' ros2 launch robot_description bring.launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch linefit_ground_segmentation_ros segmentation.launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch imu_complementary_filter complementary_filter.launch.py  ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch livox_ros_driver2 msg_MID360_launch.py   ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch fast_lio mapping.launch.py    ; exec bash;'"
# "sleep 1
"gnome-terminal -- bash -c ' ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py ; exec bash;'"
"gnome-terminal -- bash -c ' ros2 launch robot_navigation2 navigation2.launch.py ; exec bash;'"
)

cd /home/feihu_sentry/shaobing/feihu_sentry
for command in "${commands[@]}"; do
  
  eval "$command"
done
# commands1=(
#   "gnome-terminal -- bash -c ' ros2 run nav2_map_server map_saver_cli -f src/install/robot_navigation2/share/robot_navigation2/maps/my_RMUL_map3 ; exec bash;'"
# )
# for command in "${commands1[@]}"; do
  
#   eval "$command"
# done








                                                                                                                  





