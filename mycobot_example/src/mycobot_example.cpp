#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mycobot_moveit_interfaces/srv/my_cobot_command.hpp"     // CHANGE

using moveit::planning_interface::MoveGroupInterface;
MoveGroupInterface *move_group_arm;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

void add( const std::shared_ptr<mycobot_moveit_interfaces::srv::MyCobotCommand::Request> request,
          std::shared_ptr<mycobot_moveit_interfaces::srv::MyCobotCommand::Response> response) 
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  
  target_pose.position.x = request->x;
  target_pose.position.y = request->y;
  target_pose.position.z = request->z;

  q.setRPY(to_radians(request->radx),to_radians(request->rady),to_radians(request->radz)); //前方に向ける
  target_pose.orientation = tf2::toMsg(q);

  move_group_arm->setPoseTarget(target_pose);
  move_group_arm->move();

  response->arm_status = "move end";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %lf" " y: %lf" " z: %lf" "radx: %ld" "rady:%ld" "radz:%ld",   
                request->x, request->y, request->z, request->radx, request->rady, request->radz);                                          
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->arm_status.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("mycobot_example", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(move_group_node);

  std::thread thread([&executor]() {executor.spin();});

  move_group_arm = new MoveGroupInterface(move_group_node,"arm_group");
  move_group_arm->setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm->setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm->setNamedTarget("init_pose");
  move_group_arm->move();

  rclcpp::Service<mycobot_moveit_interfaces::srv::MyCobotCommand>::SharedPtr service =                 
    move_group_node->create_service<mycobot_moveit_interfaces::srv::MyCobotCommand>("my_cobot_ints",  &add);

  thread.join();
}