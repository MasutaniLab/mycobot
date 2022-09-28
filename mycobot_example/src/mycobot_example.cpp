#include <cmath>
#include <iostream>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("mycobot_example", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_node,"arm_group");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  move_group_arm.setNamedTarget("init_pose");
  move_group_arm.move();

  //単体動作確認用
  int s;

  //必要な変数を宣言
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  while(s != 0){
    std::cin >> s;

    if(s == 1){
      //準備体勢への移行
      target_pose.position.x = 0.200;
      target_pose.position.y = 0.000;
      target_pose.position.z = 0.200;

      q.setRPY(to_radians(-90),to_radians(0),to_radians(-90)); //前方に向ける
      target_pose.orientation = tf2::toMsg(q);
      move_group_arm.setPoseTarget(target_pose);
      move_group_arm.move();
      std::cout << "Idle" << std::endl;
    }
    else if(s == 2){
      //ものを把持する位置に移動
  
      target_pose.position.x = 0.200;
      target_pose.position.y = 0.000;
      target_pose.position.z = 0.170;

      q.setRPY(to_radians(-180),to_radians(0),to_radians(-90)); //下方に向ける
      target_pose.orientation = tf2::toMsg(q);
      move_group_arm.setPoseTarget(target_pose);
      move_group_arm.move();
      std::cout << "Pick" << std::endl;
    }
    else if(s == 3){
      //ものを降ろす位置に移動

      target_pose.position.x = 0.000;
      target_pose.position.y = -0.200;
      target_pose.position.z = 0.170;
      
      q.setRPY(to_radians(-180),to_radians(0),to_radians(-90)); //下方に向ける
      target_pose.orientation = tf2::toMsg(q);
      move_group_arm.setPoseTarget(target_pose);
      move_group_arm.move();
      std::cout << "Prace" << std::endl;

    }
  }

  move_group_arm.setNamedTarget("init_pose");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}