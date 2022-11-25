#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "airobot_interfaces/srv/string_command.hpp"     // CHANGE

using moveit::planning_interface::MoveGroupInterface;
MoveGroupInterface *move_group_arm;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

void callback( const std::shared_ptr<airobot_interfaces::srv::StringCommand::Request> request,
          std::shared_ptr<airobot_interfaces::srv::StringCommand::Response> response) 
{
  //リクエストを分割
  std::string cmd = request->command;
  std::string c;
  std::vector<std::string> commands;

  std::stringstream scmd{cmd}; 

  while( std::getline(scmd, c, ' ') ) {
    commands.push_back(c);
  }

  //Debug
  for (const auto& s : commands) {
    std::cout << s << std::endl;
  }

  if(commands[0] == "TargetMove"){
    if(commands.size() != 7){
      response->answer = "引数の数が正しくありません";
    }
    else{
      geometry_msgs::msg::Pose target_pose;
      tf2::Quaternion q;

      target_pose.position.x = std::stod(commands[1]);
      target_pose.position.y = std::stod(commands[2]);
      target_pose.position.z = std::stod(commands[3]);

      q.setRPY(to_radians(std::stod(commands[4])),to_radians(std::stod(commands[5])),to_radians(std::stod(commands[6])));
      target_pose.orientation = tf2::toMsg(q);

      move_group_arm->setPoseTarget(target_pose);
      move_group_arm->move();

      response->answer = "移動完了";

    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->answer.c_str());
  }

  else if (commands[0] == "LastJointSet"){
    if(commands.size() != 2){
      response->answer = "引数の数が正しくありません";
    }
    else{
      auto joint_values = move_group_arm->getCurrentJointValues();
      joint_values[5] = to_radians(std::stod(commands[1]));
      move_group_arm->setJointValueTarget(joint_values);
      move_group_arm->move();
      
      response->answer = "移動完了";
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->answer.c_str());
  }

  else{
     response->answer = "不正なコマンドです";
     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->answer.c_str());
  }
  /* 
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  
  target_pose.position.x = request->x;
  target_pose.position.y = request->y;
  target_pose.position.z = request->z;

  q.setRPY(to_radians(request->roll),to_radians(request->pitch),to_radians(request->yaw)); //前方に向ける
  target_pose.orientation = tf2::toMsg(q);

  move_group_arm->setPoseTarget(target_pose);
  move_group_arm->move();

  response->arm_status = "move end";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %lf" " y: %lf" " z: %lf" "roll: %ld" "pitch:%ld" "yaw:%ld",   
                request->x, request->y, request->z, request->roll, request->pitch, request->yaw);                                          
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->answer.c_str());
  */
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

  rclcpp::Service<airobot_interfaces::srv::StringCommand>::SharedPtr service = move_group_node->create_service<airobot_interfaces::srv::StringCommand>("my_cobot",  &callback);

  thread.join();
}