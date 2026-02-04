#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  {
    publisher_ = this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped
    >("/initialpose", 10);

    timer_ = this->create_wall_timer(
      2s,
      std::bind(&InitialPosePublisher::publish_initial_pose, this)
    );
  }

private:
  void publish_initial_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    // set initial pose as the same as the position you are spawning the robot
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 5.0;
    msg.pose.pose.position.z = 0.0;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.covariance = {
      0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.01
    };

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Initial pose published");

    timer_->cancel();  
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
