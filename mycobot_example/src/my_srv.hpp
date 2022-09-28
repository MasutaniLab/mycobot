#include <rclcpp/rclcpp.hpp>
#include "mycobot_moveit_interfaces/srv/MyCobotCommand.hpp"

class MySrv : public rclcpp::Node{
private:
  rclcpp::Service<mycobot_moveit_interfaces::srv::MyCobotCommand>::SharedPtr srv_;
  void handleService_(
    const std::shared_ptr<mycobot_moveit_interfaces::srv::MyCobotCommand::Request> request,
    const std::shared_ptr<mycobot_moveit_interfaces::srv::MyCobotCommand::Response> response
  );
public:
  MySrv(
    const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
};
