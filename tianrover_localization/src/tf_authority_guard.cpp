#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class AuthorityGuard : public rclcpp::Node {
 public:
  AuthorityGuard() : Node("tf_authority_guard") {
    odom_ = declare_parameter<std::string>("owned_odom_frame", "");
    base_ = declare_parameter<std::string>("base_frame", "base_footprint");
    expected_node_ = declare_parameter<std::string>("expected_node", "kiss_icp_node");
    any_parent_ = declare_parameter<bool>("check_any_parent", false);
    output_ = create_publisher<std_msgs::msg::Bool>("lidar/tf_authority_ok", rclcpp::QoS(1).transient_local());
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        if (odom_.empty()) return;
        for (const auto & tf : msg->transforms) {
          if ((any_parent_ || tf.header.frame_id == odom_) && tf.child_frame_id == base_) {
            Gid gid;
            const auto & raw = info.get_rmw_message_info().publisher_gid;
            std::copy(raw.data, raw.data + RMW_GID_STORAGE_SIZE, gid.begin());
            authorities_[gid] = std::chrono::steady_clock::now();
          }
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { check(); });
  }
 private:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  void check() {
    bool valid = true;
    const auto now = std::chrono::steady_clock::now();
    const auto endpoints = get_publishers_info_by_topic("/tf");
    for (auto it = authorities_.begin(); it != authorities_.end();) {
      if (now - it->second > std::chrono::seconds(2)) {
        it = authorities_.erase(it);
        continue;
      }
      bool expected = false;
      for (const auto & publisher : endpoints) {
        if (publisher.endpoint_gid() == it->first && publisher.node_name() == expected_node_ &&
            publisher.node_namespace() == get_namespace()) expected = true;
      }
      valid = valid && expected;
      ++it;
    }
    std_msgs::msg::Bool message;
    message.data = valid;
    output_->publish(message);
    if (!valid) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Conflicting odom -> base TF publisher; stop old wheel/EKF frontend");
  }
  std::string odom_, base_, expected_node_;
  bool any_parent_;
  std::map<Gid, std::chrono::steady_clock::time_point> authorities_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr output_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AuthorityGuard>());
  rclcpp::shutdown();
  return 0;
}
