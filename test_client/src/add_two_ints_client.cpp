#include "rclcpp/rclcpp.hpp"
#include "mycobot_moveit_interfaces/srv/my_cobot_command.hpp"     

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 7) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "引数の数が不正");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("my_cobot_ints_client"); 
  rclcpp::Client<mycobot_moveit_interfaces::srv::MyCobotCommand>::SharedPtr client =                        
    node->create_client<mycobot_moveit_interfaces::srv::MyCobotCommand>("my_cobot_ints");                  

  auto request = std::make_shared<mycobot_moveit_interfaces::srv::MyCobotCommand::Request>();               
  request->x = atof(argv[1]);
  request->y = atof(argv[2]);
  request->z = atof(argv[3]);
  request->roll = atof(argv[4]);
  request->pitch = atof(argv[5]);
  request->yaw = atof(argv[6]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s", result.get()->arm_status.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service my_cobot_ints");    
  }

  rclcpp::shutdown();
  return 0;
}