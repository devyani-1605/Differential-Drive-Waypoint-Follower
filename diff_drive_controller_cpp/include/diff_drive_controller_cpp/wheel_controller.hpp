#ifndef DIFF_DRIVE_ROBOT_WHEEL_CONTROLLER_HPP
#define DIFF_DRIVE_ROBOT_WHEEL_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

namespace diff_drive_robot {

class WheelController : public rclcpp::Node {
public:
    WheelController();

private:
    // Robot parameters
    double wheel_radius_;      // meters
    double wheel_separation_;  // meters
    double max_rpm_;          // Maximum RPM for the motors

    // Subscribers and publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;

    // Callback for velocity commands
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Utility functions
    double convertVelocityToRPM(double linear_velocity);
    void loadParameters();
    void declareParameters();
};

}  // namespace diff_drive_robot

#endif  // DIFF_DRIVE_ROBOT_WHEEL_CONTROLLER_HPP