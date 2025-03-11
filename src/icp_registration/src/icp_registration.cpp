#include "icp_registration/icp_registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>

#include <fstream>  // 文件操作
#include <sstream>  // 字符串流
#include <string>   // 字符串操作

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
    // : Node("icp_registration", options), rough_iter_(120), refine_iter_(100),
      first_scan_(true) {
  is_ready_ = false;
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // this->declare_parameter("use_sim_time", false);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // Read the pcd file
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  reader.read(pcd_path_, *cloud);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  // Add normal to the pointcloud
  refine_map_ = addNorm(cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = addNorm(filterd_point_rough);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  // thresh_ = this->declare_parameter("thresh", 0.85);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));
  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Set up the timer
  // timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
  //   if (is_ready_) {
  //     map_to_odom_.header.stamp = now();
  //     map_to_odom_.header.frame_id = map_frame_id_;
  //     map_to_odom_.child_frame_id = odom_frame_id_;
  //     tf_broadcaster_->sendTransform(map_to_odom_);
  //   }
  // });

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Publishing tf"
                                 << map_to_odom_.transform.translation.x << " "
                                 << map_to_odom_.transform.translation.y << " "
                                 << map_to_odom_.transform.translation.z);
          map_to_odom_.header.stamp = now();
          map_to_odom_.header.frame_id = map_frame_id_;
          map_to_odom_.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(map_to_odom_);
        }
        //is_ready_=false;
      }
      rate.sleep();
    }
  });


  // tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
  //   rclcpp::Rate rate(100);
  //   while (rclcpp::ok()) {
  //     {
  //       std::lock_guard lock(mutex_);
  //       if (is_ready_) {
  //         RCLCPP_INFO_STREAM(this->get_logger(),
  //                            "Publishing tf"
  //                            << map_to_odom_.transform.translation.x << " "
  //                            << map_to_odom_.transform.translation.y << " "
  //                            << map_to_odom_.transform.translation.z);

  //         // 只在第一次写入数据时进行文件操作
  //         if (is_first_write_) {
  //           std::string file_path = "/home/hdm/shaobing/feihu/install/robot_navigation2/share/robot_navigation2/maps/cf2_new100hz.yaml";  // 替换为实际路径
  //           std::ifstream infile(file_path);
  //           if (!infile.is_open()) {
  //             RCLCPP_ERROR(this->get_logger(), "Failed to open file for reading.");
  //             return;
  //           }

  //           std::stringstream buffer;
  //           buffer << infile.rdbuf();
  //           std::string file_content = buffer.str();
            
  //           // 构造新的 origin 值
  //           std::string new_origin = "origin: [" + std::to_string(map_to_odom_.transform.translation.x) + ", "
  //                                                  + std::to_string(map_to_odom_.transform.translation.y) + ", "
  //                                                  + std::to_string(map_to_odom_.transform.translation.z) + "]";
            
  //           // 替换 origin 行
  //           size_t pos = file_content.find("origin:");
  //           if (pos != std::string::npos) {
  //             size_t end_pos = file_content.find("\n", pos);  // 寻找这一行的结束
  //             file_content.replace(pos, end_pos - pos, new_origin);
  //           }

  //           // 写回修改后的内容到文件
  //           std::ofstream outfile(file_path);
  //           if (!outfile.is_open()) {
  //             RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
  //             return;
  //           }
  //           outfile << file_content;
  //           outfile.close();  // 写入完成后关闭文件

  //           // 设置标志为 false，避免后续重复写入
  //           is_first_write_ = false;
  //         }

  //         // 继续发布 tf
  //         map_to_odom_.header.stamp = now();
  //         map_to_odom_.header.frame_id = map_frame_id_;
  //         map_to_odom_.child_frame_id = odom_frame_id_;
  //         tf_broadcaster_->sendTransform(map_to_odom_);
  //       }
  //     }
  //     rate.sleep();
  //   }
  // });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  if (tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::fromROSMsg(*msg, *cloud_in_);
  if (first_scan_) {
    auto pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose.pose = initial_pose_;
    initialPoseCallback(pose_msg);
    first_scan_ = false;
  }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  // 从消息中获取初始位姿并构建一个4x4的初始变换矩阵
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);  // 提取位置
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);  // 提取姿态（四元数）
  
  Eigen::Matrix4d initial_guess;  // 创建一个4x4矩阵来存储初始的变换矩阵
  initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();  // 设置旋转部分
  initial_guess.block<3, 1>(0, 3) = pos;  // 设置平移部分
  initial_guess(3, 3) = 1;  // 设置齐次坐标的最后一项

  // 使用ICP对点云进行配准
  RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");  // 输出日志信息
  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);  // 使用多传感器同步配准
  // Eigen::Matrix4d result = align(cloud_in_, initial_guess);  // 另一种ICP配准方法（注释掉）
  
  // 如果ICP配准失败，使用初始猜测
  if (!success_) {
    map_to_laser = initial_guess;  // ICP失败时使用初始猜测作为结果
    RCLCPP_ERROR(this->get_logger(), "ICP failed");  // 输出错误日志
  }

  // 初始化激光到里程计的变换矩阵
  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();  // 初始化为单位矩阵
  try {
    // 获取从激光帧到里程计帧的变换
    auto transform =
        tf_buffer_->lookupTransform(laser_frame_id_, odom_frame_id_, now(),
                                    rclcpp::Duration::from_seconds(10));  // 查找变换
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);  // 提取平移部分
    Eigen::Quaterniond q(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);  // 提取旋转部分
    
    // 更新激光到里程计的变换矩阵
    laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();  // 设置旋转部分
    laser_to_odom.block<3, 1>(0, 3) = t;  // 设置平移部分
  } catch (tf2::TransformException &ex) {
    // 如果获取变换失败，输出错误并标记为不可用
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    is_ready_ = false;  // 标记系统未准备好
    return;  // 退出函数
  }
  
  // 计算从地图到里程计的最终变换矩阵
  Eigen::Matrix4d result = map_to_laser * laser_to_odom.matrix().cast<double>();  // 乘积计算变换矩阵

  // 打印调试信息
  std::cout << "map_to_laser = \n" << map_to_laser << std::endl;
  std::cout << "laser_to_odom = \n" << laser_to_odom << std::endl;

  // 更新成员变量map_to_odom_中的变换信息
  std::lock_guard lock(mutex_);  // 锁定mutex，确保线程安全
  map_to_odom_.transform.translation.x = result(0, 3);  // 设置平移部分
  map_to_odom_.transform.translation.y = result(1, 3);
  map_to_odom_.transform.translation.z = result(2, 3);

  // 提取旋转部分并更新四元数
  Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
  q = Eigen::Quaterniond(rotation);

  map_to_odom_.transform.rotation.w = q.w();
  map_to_odom_.transform.rotation.x = q.x();
  map_to_odom_.transform.rotation.y = q.y();
  map_to_odom_.transform.rotation.z = q.z();
  is_ready_ = true;  // 标记系统已准备好

  // 下面的代码被注释掉了，避免重复更新map_to_odom_的值：
  // std::lock_guard lock(mutex_);
  // map_to_odom_.transform.translation.x = map_to_laser(0, 3);
  // map_to_odom_.transform.translation.y = map_to_laser(1, 3);
  // map_to_odom_.transform.translation.z = map_to_laser(2, 3);
  //
  // Eigen::Matrix3d rotation = map_to_laser.block<3, 3>(0, 0);
  // q = Eigen::Quaterniond(rotation);
  //
  // map_to_odom_.transform.rotation.w = q.w();
  // map_to_odom_.transform.rotation.x = q.x();
  // map_to_odom_.transform.rotation.y = q.y();
  // map_to_odom_.transform.rotation.z = q.z();
  // is_ready_ = true;

}


// Eigen::Matrix4d IcpNode::align(PointCloudXYZI::Ptr source,
//                                const Eigen::Matrix4d &init_guess) {
//   success_ = false;
//   // Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);

//   pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
//       new pcl::PointCloud<pcl::PointXYZI>);

//   voxel_rough_filter_.setInputCloud(source);
//   voxel_rough_filter_.filter(*rough_source);
//   voxel_refine_filter_.setInputCloud(source);
//   voxel_refine_filter_.filter(*refine_source);

//   PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
//   PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
//   PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);
//   auto tic = std::chrono::system_clock::now();
//   icp_rough_.setInputSource(rough_source_norm);
//   icp_rough_.align(*align_point, init_guess.cast<float>());

//   score_ = icp_rough_.getFitnessScore();
//   if (!icp_rough_.hasConverged())
//     return Eigen::Matrix4d::Zero();

//   icp_refine_.setInputSource(refine_source_norm);
//   icp_refine_.align(*align_point, icp_rough_.getFinalTransformation());
//   score_ = icp_refine_.getFitnessScore();

//   if (!icp_refine_.hasConverged())
//     return Eigen::Matrix4d::Zero();
//   if (score_ > thresh_)
//     return Eigen::Matrix4d::Zero();
//   success_ = true;
//   auto toc = std::chrono::system_clock::now();
//   std::chrono::duration<double> duration = toc - tic;
//   RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() *
//   1000); RCLCPP_INFO(this->get_logger(), "score: %f", score_);

//   return icp_refine_.getFinalTransformation().cast<double>();
// }

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -yaw_offset_; k <= yaw_offset_; k++) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
      new pcl::PointCloud<pcl::PointXYZI>);

  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
  PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
  PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  auto tic = std::chrono::system_clock::now();
  for (Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.setInputSource(rough_source_norm);
    icp_rough_.align(*align_point, init_pose);
    if (!icp_rough_.hasConverged())
      continue;
    double rough_score = icp_rough_.getFitnessScore();
    if (rough_score > 2 * thresh_)
      continue;
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  if (!rough_converge)
    return Eigen::Matrix4d::Zero();

  icp_refine_.setInputSource(refine_source_norm);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();

  if (!icp_refine_.hasConverged())
    return Eigen::Matrix4d::Zero();
  if (score_ > thresh_)
    return Eigen::Matrix4d::Zero();
  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}


// Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
//                                         const Eigen::Matrix4d &init_guess) {
//   // 定义一个lambda函数，将旋转矩阵转为欧拉角（roll, pitch, yaw）
//   static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
//     double roll = std::atan2(rot(2, 1), rot(2, 2));  // 计算roll角
//     double pitch = asin(-rot(2, 0));  // 计算pitch角
//     double yaw = std::atan2(rot(1, 0), rot(0, 0));  // 计算yaw角
//     return Eigen::Vector3d(roll, pitch, yaw);  // 返回欧拉角
//   };

//   success_ = false;  // 初始化成功标志为false
//   // 从初始猜测矩阵中提取平移部分和旋转部分
//   Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);  // 提取平移向量
//   Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);  // 提取旋转矩阵
//   Eigen::Vector3d rpy = rotate2rpy(rotation);  // 将旋转矩阵转换为欧拉角
//   Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());  // 创建绕X轴的旋转
//   Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());  // 创建绕Y轴的旋转
//   std::vector<Eigen::Matrix4f> candidates;  // 用于存储候选位姿的列表
//   Eigen::Matrix4f temp_pose;  // 临时矩阵，表示每个候选位姿

//   // 输出初始猜测信息
//   RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
//               xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

//   // 在平移和旋转角度范围内进行遍历，生成多个候选位姿
//   for (int i = -1; i <= 1; i++) {
//     for (int j = -1; j <= 1; j++) {
//       for (int k = -yaw_offset_; k <= yaw_offset_; k++) {
//         Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
//                             xyz(2));  // 生成平移向量
//         Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
//                                    Eigen::Vector3f::UnitZ());  // 生成绕Z轴的旋转
//         temp_pose.setIdentity();  // 初始化temp_pose为单位矩阵
//         temp_pose.block<3, 3>(0, 0) =
//             (rollAngle * pitchAngle * yawAngle).toRotationMatrix();  // 设置旋转部分
//         temp_pose.block<3, 1>(0, 3) = pos;  // 设置平移部分
//         candidates.push_back(temp_pose);  // 将候选位姿加入列表
//       }
//     }
//   }

//   // 创建指向源点云的指针
//   pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(new pcl::PointCloud<pcl::PointXYZI>);

//   // 使用体素滤波器对源点云进行粗滤波和精滤波
//   voxel_rough_filter_.setInputCloud(source);
//   voxel_rough_filter_.filter(*rough_source);
//   voxel_refine_filter_.setInputCloud(source);
//   voxel_refine_filter_.filter(*refine_source);

//   // 计算带法向量的点云
//   PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
//   PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
//   PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

//   Eigen::Matrix4f best_rough_transform;  // 存储最佳粗对齐变换
//   double best_rough_score = 10.0;  // 初始化最佳粗对齐得分
//   bool rough_converge = false;  // 初始化粗对齐是否收敛标志
//   auto tic = std::chrono::system_clock::now();  // 记录开始时间

//   // 对每个候选位姿进行粗对齐
//   for (Eigen::Matrix4f &init_pose : candidates) {
//     icp_rough_.setInputSource(rough_source_norm);  // 设置源点云
//     icp_rough_.align(*align_point, init_pose);  // 进行ICP对齐
//     if (!icp_rough_.hasConverged())  // 如果没有收敛，则跳过
//       continue;
//     double rough_score = icp_rough_.getFitnessScore();  // 获取粗对齐得分
//     if (rough_score > 2 * thresh_)  // 如果粗对齐得分超过阈值，则跳过
//       continue;
//     if (rough_score < best_rough_score) {  // 如果当前得分更好，更新最佳结果
//       best_rough_score = rough_score;
//       rough_converge = true;
//       best_rough_transform = icp_rough_.getFinalTransformation();  // 更新最佳变换矩阵
//     }
//   }

//   if (!rough_converge)  // 如果粗对齐没有收敛，返回零矩阵
//     return Eigen::Matrix4d::Zero();

//   // 进行精细对齐
//   icp_refine_.setInputSource(refine_source_norm);  // 设置精细对齐的源点云
//   icp_refine_.align(*align_point, best_rough_transform);  // 进行精细对齐
//   score_ = icp_refine_.getFitnessScore();  // 获取精细对齐得分

//   if (!icp_refine_.hasConverged())  // 如果精细对齐没有收敛，返回零矩阵
//     return Eigen::Matrix4d::Zero();
//   if (score_ > 1* thresh_)  // 如果得分超过阈值，返回零矩阵
//     return Eigen::Matrix4d::Zero();
  
//   success_ = true;  // 标记对齐成功
//   auto toc = std::chrono::system_clock::now();  // 记录结束时间
//   std::chrono::duration<double> duration = toc - tic;  // 计算对齐耗时
//   RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);  // 输出对齐耗时
//   RCLCPP_INFO(this->get_logger(), "score: %f", score_);  // 输出精细对齐得分

//   // 返回最终的精细对齐变换矩阵
//   return icp_refine_.getFinalTransformation().cast<double>();
// }



PointCloudXYZIN::Ptr
IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);
  PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
