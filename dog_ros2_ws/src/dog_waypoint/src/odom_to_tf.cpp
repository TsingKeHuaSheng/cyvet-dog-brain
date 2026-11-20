/**
 * odom_to_tf.cpp
 * ========================================================
 * 把 /odom 转成 TF（odom -> base_link）。
 *
 * 为什么需要它：
 *   uniubi_motion_bridge 明确"当前不发布 TF"，只发 /odom 消息。
 *   而 Nav2、RViz、robot_state_publisher 的整条 TF 树都需要
 *   odom -> base_link 这一环。少了它，Nav2 会一直报
 *   "Timed out waiting for transform from base_link to odom"。
 *
 * 里程计的 position/yaw 已经是设备端累计的位姿，直接拿来做 TF 即可，
 * 不要再积分一次。
 *
 * 用法：
 *   ros2 run dog_waypoint odom_to_tf --ros-args \
 *     -p odom_frame:=odom -p base_frame:=base_link
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTf : public rclcpp::Node {
public:
    OdomToTf() : rclcpp::Node("odom_to_tf") {
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(*msg); });

        RCLCPP_INFO(get_logger(), "发布 TF: %s -> %s",
                    odom_frame_.c_str(), base_frame_.c_str());
    }

private:
    void onOdom(const nav_msgs::msg::Odometry& msg) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg.header.stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;

        tf.transform.translation.x = msg.pose.pose.position.x;
        tf.transform.translation.y = msg.pose.pose.position.y;
        tf.transform.translation.z = msg.pose.pose.position.z;
        tf.transform.rotation = msg.pose.pose.orientation;

        broadcaster_->sendTransform(tf);
    }

    std::string odom_frame_;
    std::string base_frame_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTf>());
    rclcpp::shutdown();
    return 0;
}
