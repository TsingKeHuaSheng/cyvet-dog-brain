/**
 * waypoint_recorder.cpp
 * ========================================================
 * "教一次"：手动/遥控把狗走一遍，这个节点按距离采样 /odom 存成航点 CSV。
 *
 * 用法：
 *   1) 一个终端跑记录器
 *        ros2 run dog_waypoint waypoint_recorder --ros-args \
 *          -p output_file:=/home/nvidia/dog_paths/my_path.csv \
 *          -p min_step_m:=0.20
 *   2) 另一个终端让狗走（低速度手动控制），例如：
 *        ros2 service call /motion/start_action uniubi_motion_bridge/srv/StartMotionAction \
 *          "{action: walking, params_json: '{\"lineVelocityX\":0.0,\"lineVelocityY\":0.0,\"velocity\":0.0}'}"
 *        ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}'
 *      （到点后把 linear.x 改回 0，或直接 Ctrl+C 结束那次 pub）
 *   3) 记录器按 Ctrl+C 退出，文件即已落盘（每采一点就写盘，不怕崩）
 *
 * 细节：
 *   - 只按"走了多远"采样，不按时间，所以快慢不影响航点密度
 *   - 里程计是设备端累计值，这里只读不积分
 *   - 转弯处会自动加密（因为位置变化累积到阈值才记）
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class WaypointRecorder : public rclcpp::Node {
public:
    WaypointRecorder() : rclcpp::Node("waypoint_recorder") {
        output_file_ = declare_parameter<std::string>(
            "output_file", "/home/nvidia/dog_paths/recorded_path.csv");
        min_step_m_ = declare_parameter<double>("min_step_m", 0.20);
        min_yaw_step_ = declare_parameter<double>("min_yaw_step_rad", 0.35);

        RCLCPP_INFO(get_logger(), "输出文件: %s", output_file_.c_str());
        RCLCPP_INFO(get_logger(), "采样阈值: 位移 %.2f m / 转角 %.2f rad",
                    min_step_m_, min_yaw_step_);
        RCLCPP_INFO(get_logger(), "开始用 /cmd_vel 低速驱动狗，走完后 Ctrl+C");

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(*msg); });
    }

private:
    void onOdom(const nav_msgs::msg::Odometry& msg) {
        const double x = msg.pose.pose.position.x;
        const double y = msg.pose.pose.position.y;
        const double yaw = yawFromQuaternion(msg.pose.pose.orientation);

        if (!started_) {
            started_ = true;
            appendPoint(x, y, yaw);
            return;
        }

        const double d = std::hypot(x - last_x_, y - last_y_);
        double dyaw = yaw - last_yaw_;
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw <= -M_PI) dyaw += 2.0 * M_PI;

        if (d < min_step_m_ && std::fabs(dyaw) < min_yaw_step_) return;
        appendPoint(x, y, yaw);
    }

    void appendPoint(double x, double y, double yaw) {
        last_x_ = x;
        last_y_ = y;
        last_yaw_ = yaw;
        points_.push_back({x, y, yaw});
        save();

        if (points_.size() % 10 == 0) {
            RCLCPP_INFO(get_logger(), "已记录 %zu 点，当前 (%.2f, %.2f, yaw %.2f)",
                        points_.size(), x, y, yaw);
        }
    }

    /// 每次追加就整体重写：文件小、抗崩溃
    void save() {
        std::ofstream out(output_file_, std::ios::trunc);
        if (!out) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "写文件失败: %s", output_file_.c_str());
            return;
        }
        out << "# x,y,yaw  由 waypoint_recorder 生成\n";
        for (const auto& p : points_) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f\n", p.x, p.y, p.yaw);
            out << buf;
        }
    }

    struct Point {
        double x;
        double y;
        double yaw;
    };

    std::string output_file_;
    double min_step_m_ = 0.20;
    double min_yaw_step_ = 0.35;

    bool started_ = false;
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_yaw_ = 0.0;
    std::vector<Point> points_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<WaypointRecorder>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
