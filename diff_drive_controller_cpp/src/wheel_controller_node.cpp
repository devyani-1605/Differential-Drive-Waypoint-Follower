#include "diff_drive_controller_cpp/wheel_controller.hpp"

namespace diff_drive_robot {

WheelController::WheelController() : Node("wheel_controller") {
    declareParameters();
    loadParameters();

    // Create subscribers and publishers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&WheelController::cmdVelCallback, this, std::placeholders::_1));

    left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "left_wheel_rpm", 10);
    right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "right_wheel_rpm", 10);

    RCLCPP_INFO(this->get_logger(), "Wheel Controller Node initialized");
}

void WheelController::declareParameters() {
    this->declare_parameter("wheel_radius", 0.1);
    this->declare_parameter("wheel_separation", 0.5);
    this->declare_parameter("max_rpm", 100.0);
}

void WheelController::loadParameters() {
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_double();
}

void WheelController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Calculate wheel velocities
    double left_velocity = msg->linear.x - (msg->angular.z * wheel_separation_ / 2.0);
    double right_velocity = msg->linear.x + (msg->angular.z * wheel_separation_ / 2.0);

    // Convert to RPM
    double left_rpm = convertVelocityToRPM(left_velocity);
    double right_rpm = convertVelocityToRPM(right_velocity);

    // Publish RPM values
    auto left_msg = std_msgs::msg::Float64();
    auto right_msg = std_msgs::msg::Float64();
    left_msg.data = left_rpm;
    right_msg.data = right_rpm;

    left_rpm_pub_->publish(left_msg);
    right_rpm_pub_->publish(right_msg);
}

double WheelController::convertVelocityToRPM(double linear_velocity) {
    // Convert linear velocity (m/s) to RPM
    double angular_velocity = linear_velocity / wheel_radius_;  // rad/s
    double rpm = (angular_velocity * 60.0) / (2.0 * M_PI);     // RPM
    
    // Clamp to maximum RPM
    return std::clamp(rpm, -max_rpm_, max_rpm_);
}

}  // namespace diff_drive_robot

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<diff_drive_robot::WheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
