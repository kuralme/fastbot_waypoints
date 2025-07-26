#include <cmath>
#include <memory>
#include <thread>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using WaypointAction = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<WaypointAction>;

class WaypointActionServer : public rclcpp::Node {
public:
  WaypointActionServer() : Node("waypoint_action_server") {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<WaypointAction>(
        this, "fastbot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 1);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionServer::odom_callback, this, _1));

    yaw_precision_ = M_PI / 90.0; // 2 degrees
    dist_precision_ = 0.05;       // 5 cm
  }

private:
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point current_pos_{};
  double roll, pitch, current_yaw_{0.0};

  double yaw_precision_;
  double dist_precision_;

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    double ox = msg->pose.pose.orientation.x;
    double oy = msg->pose.pose.orientation.y;
    double oz = msg->pose.pose.orientation.z;
    double ow = msg->pose.pose.orientation.w;

    tf2::Quaternion q(ox, oy, oz, ow);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, current_yaw_);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received waypoint goal: (%.2f, %.2f)",
                goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    RCLCPP_INFO(get_logger(), "Goal canceled");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{
        std::bind(&WaypointActionServer::execute, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();
    const auto goal = goal_handle->get_goal();
    auto des = goal->position;

    rclcpp::Rate loop_rate(25);

    enum class State { FIX_YAW, GO_TO_POINT, FIX_FINAL_YAW };
    State state = State::FIX_YAW;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        twist_pub_->publish(geometry_msgs::msg::Twist());
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      double err_pos =
          std::hypot(des.y - current_pos_.y, des.x - current_pos_.x);
      double desired_yaw =
          std::atan2(des.y - current_pos_.y, des.x - current_pos_.x);

      double fixed_yaw_offset = 1.47;
      double global_yaw = current_yaw_ - fixed_yaw_offset;
      double err_yaw = desired_yaw - global_yaw;
      if (err_yaw > M_PI)
        err_yaw -= 2 * M_PI;
      if (err_yaw < -M_PI)
        err_yaw += 2 * M_PI;

      geometry_msgs::msg::Twist twist;

      if (state == State::FIX_YAW) {
        if (std::fabs(err_yaw) > yaw_precision_) {
          RCLCPP_INFO(get_logger(), "Fixing yaw...");
          twist.angular.z = (err_yaw > 0) ? 0.3 : -0.3;
          feedback->state = "fix yaw";
        } else {
          twist.angular.z = 0.0;
          state = State::GO_TO_POINT;
          continue;
        }

      } else if (state == State::GO_TO_POINT) {
        if (err_pos <= dist_precision_ / 2.) {
          // Reached position, now fix final yaw before stopping
          state = State::FIX_FINAL_YAW;
          continue;
        }
        if (std::fabs(err_yaw) > yaw_precision_) {
          state = State::FIX_YAW;
          continue;
        }
        RCLCPP_INFO(get_logger(), "Moving to goal...");
        twist.linear.x = 0.5;
        feedback->state = "go to point";

      } else if (state == State::FIX_FINAL_YAW) {
        if (std::fabs(err_yaw) > yaw_precision_) {
          RCLCPP_INFO(get_logger(), "Fixing final yaw...");
          twist.linear.x = 0.0;
          twist.angular.z = (err_yaw > 0) ? 0.1 : -0.1;
          feedback->state = "fix final yaw";
        } else {
          twist.angular.z = 0.0;
          // Final yaw fixed, goal complete
          break;
        }
      }

      twist_pub_->publish(twist);
      feedback->position = current_pos_;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Stop motion
    twist_pub_->publish(geometry_msgs::msg::Twist());
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal reached and yaw aligned.");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointActionServer>());
  rclcpp::shutdown();
  return 0;
}
