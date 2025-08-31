// Copyright 2025 Yadunund Vijay.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace rmw_zenoh_examples
{

class OdomSubscription : public rclcpp::Node
{
public:
  OdomSubscription(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("odom_subscription", options),
    message_count_(0),
    first_message_time_(),
    last_message_time_()
  {
    frequency_report_interval_ =
      this->declare_parameter("frequency_report_interval", 20);

    rclcpp::QoS odom_qos(100);
    odom_qos.reliable().durability_volatile();
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      odom_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
      {
        const auto now = this->now();
        message_count_++;

        if (message_count_ == 1) {
          first_message_time_ = now;
          last_message_time_ = now;
          RCLCPP_INFO(this->get_logger(), "First odometry message received");
          return;
        }

        // Calculate average frequency since start
        const auto total_time = (now - first_message_time_).seconds();
        const double average_hz = (total_time > 0.0) ? (message_count_ - 1) / total_time : 0.0;

        last_message_time_ = now;

        // Report frequency at intervals
        if (message_count_ % frequency_report_interval_ == 0) {
          RCLCPP_INFO(
            this->get_logger(),
            "Average odom frequency over last %d messages: %.2f Hz (total messages: %lu)",
            frequency_report_interval_, average_hz, message_count_
          );
        }
      }
    );

    RCLCPP_INFO(this->get_logger(),
        "Odometry subscription node started. Will report frequency every %d messages.",
        frequency_report_interval_);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  uint64_t message_count_;
  rclcpp::Time first_message_time_;
  rclcpp::Time last_message_time_;
  // Report frequency every N messages.
  uint32_t frequency_report_interval_;
};
}  // namespace rmw_zenoh_examples

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rmw_zenoh_examples::OdomSubscription)
