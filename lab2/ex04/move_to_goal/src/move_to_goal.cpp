#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class MoveToGoal : public rclcpp::Node {
public:
  MoveToGoal() : Node("move_to_goal") {
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    declare_parameter<double>("theta", 0.0);

    goal_x_ = get_parameter("x").as_double();
    goal_y_ = get_parameter("y").as_double();
    goal_theta_ = get_parameter("theta").as_double();

    pose_sub_ = create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&MoveToGoal::pose_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MoveToGoal::control_loop, this));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_theta_ = msg->theta;
  }

  void control_loop() {
    double distance = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    double angle_error = angle_to_goal - current_theta_;

    // Normalize angle
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;
    if (distance > 0.1) {
      cmd.linear.x = 2.0 * distance; // Proportional linear velocity
      cmd.angular.z = 4.0 * angle_error; // Proportional angular velocity
    } else if (std::abs(goal_theta_ - current_theta_) > 0.05) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 2.0 * (goal_theta_ - current_theta_); // Orient to goal theta
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      rclcpp::shutdown();
    }
    cmd_vel_pub_->publish(cmd);
  }

  double goal_x_, goal_y_, goal_theta_;
  double current_x_, current_y_, current_theta_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToGoal>());
  rclcpp::shutdown();
  return 0;
}
