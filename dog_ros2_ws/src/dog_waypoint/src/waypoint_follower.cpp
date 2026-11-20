/**
 * waypoint_follower.cpp
 * ========================================================
 * 让 Cyvet 机器狗按既定路径行走的最小可用 C++ 节点。
 *
 * 它做的事（就是一个状态机）：
 *   1. 等 /odom 和 /motion/start_action 就绪
 *   2. 调 /motion/start_action("walking") —— bridge 会自动取控制权
 *   3. 以 control_rate_hz 的节奏读 /odom、算控制量、发 /cmd_vel
 *   4. 每个航点用"到点判定"推进，最后一个点到达后：
 *        清零速度 -> /motion/stop_action -> /motion/release_control -> 退出
 *   5. 任何一条安全条件触发就立刻停车走同样的收尾（不会把狗丢在行走状态）
 *
 * 为什么要"条件驱动"而不是"定时发速度"：
 *   /cmd_vel 只是把速度参数交给当前动作，本身不闭环。路径行走必须由
 *   /odom 的位姿反馈来决策——这就是"具体条件下执行相应指令"的本体。
 *
 * 安全条件（任一命中即 abort）：
 *   - 里程计丢帧超过 odom_timeout_s
 *   - 总耗时超过 max_duration_s
 *   - 电量低于 battery_min
 *   - /motion/status 报控制权丢失（control_state != CONTROLLED）
 *   - /motion/status 报 last_error_code != 0
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uniubi_motion_bridge/msg/motion_status.hpp"
#include "uniubi_motion_bridge/srv/start_motion_action.hpp"

using namespace std::chrono_literals;

namespace {

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool has_yaw = false;
};

/// 把角度归一到 (-pi, pi]
double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

/// 从四元数取 yaw（不依赖 tf2，减少依赖面）
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

template <typename T>
T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
}

/// 读 CSV：每行 "x,y" 或 "x,y,yaw"；# 开头为注释
std::vector<Waypoint> loadPath(const std::string& file) {
    std::vector<Waypoint> out;
    std::ifstream in(file);
    if (!in) return out;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string token;
        std::vector<double> nums;
        while (std::getline(ss, token, ',')) {
            try {
                nums.push_back(std::stod(token));
            } catch (...) {
                nums.clear();
                break;
            }
        }
        if (nums.size() < 2) continue;
        Waypoint wp;
        wp.x = nums[0];
        wp.y = nums[1];
        if (nums.size() >= 3) {
            wp.yaw = nums[2];
            wp.has_yaw = true;
        }
        out.push_back(wp);
    }
    return out;
}

}  // namespace

class WaypointFollower : public rclcpp::Node {
public:
    WaypointFollower() : rclcpp::Node("waypoint_follower") {
        // ---------- 参数 ----------
        path_file_ = declare_parameter<std::string>("path_file", "");
        max_duration_s_ = declare_parameter<double>("max_duration_s", 120.0);
        battery_min_ = declare_parameter<double>("battery_min", 0.15);
        odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 1.0);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

        v_max_ = declare_parameter<double>("v_max", 0.25);
        w_max_ = declare_parameter<double>("w_max", 0.6);
        k_rho_ = declare_parameter<double>("k_rho", 0.8);
        k_alpha_ = declare_parameter<double>("k_alpha", 1.5);
        pos_tol_ = declare_parameter<double>("pos_tolerance", 0.15);
        yaw_tol_ = declare_parameter<double>("yaw_tolerance", 0.15);
        loop_ = declare_parameter<bool>("loop", false);
        start_action_ = declare_parameter<std::string>("start_action", "walking");

        // ---------- 路径 ----------
        if (path_file_.empty()) {
            RCLCPP_FATAL(get_logger(), "必须指定 path_file 参数");
            throw std::runtime_error("path_file is required");
        }
        waypoints_ = loadPath(path_file_);
        if (waypoints_.empty()) {
            RCLCPP_FATAL(get_logger(), "路径文件为空或格式不对: %s", path_file_.c_str());
            throw std::runtime_error("empty path");
        }
        RCLCPP_INFO(get_logger(), "已加载 %zu 个航点，来自 %s",
                    waypoints_.size(), path_file_.c_str());

        // ---------- 通信 ----------
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                pose_x_ = msg->pose.pose.position.x;
                pose_y_ = msg->pose.pose.position.y;
                pose_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
                last_odom_ = now();
                have_odom_ = true;
            });

        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
                battery_ = msg->percentage;  // 0.0 ~ 1.0
                have_battery_ = true;
            });

        status_sub_ = create_subscription<uniubi_motion_bridge::msg::MotionStatus>(
            "/motion/status", rclcpp::QoS(10).transient_local(),
            [this](uniubi_motion_bridge::msg::MotionStatus::SharedPtr msg) {
                control_state_ = msg->control_state;
                last_error_code_ = msg->last_error_code;
                last_error_message_ = msg->last_error_message;
                have_status_ = true;
            });

        start_cli_ = create_client<uniubi_motion_bridge::srv::StartMotionAction>(
            "/motion/start_action");
        stop_cli_ = create_client<std_srvs::srv::Trigger>("/motion/stop_action");
        release_cli_ = create_client<std_srvs::srv::Trigger>("/motion/release_control");

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_hz_),
            [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "参数: v_max=%.2f w_max=%.2f 到点容差=%.2f m 总超时=%.0f s",
                    v_max_, w_max_, pos_tol_, max_duration_s_);
    }

private:
    enum class State { kWaiting, kStarting, kDriving, kFinishing, kDone };

    // ---------- 主循环 ----------
    void tick() {
        switch (state_) {
            case State::kWaiting:
                waiting();
                break;
            case State::kDriving:
                driving();
                break;
            default:
                break;  // 其余状态由异步回调推进
        }
    }

    void waiting() {
        if (now() < retry_after_) return;  // 失败后的退避等待

        if (!start_cli_->service_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "等待 /motion/start_action 服务（bridge 起来了吗？）");
            return;
        }
        // 注意：这里绝对不能等 /odom！
        // /odom 只在 Walk 动作期间才有，而 Walk 要先把动作启动起来才会进入，
        // 先等 odom 就成了死锁。正确顺序是：先启动动作 -> 再等里程计。
        beginStart();
    }

    void beginStart() {
        auto req = std::make_shared<uniubi_motion_bridge::srv::StartMotionAction::Request>();
        req->action = start_action_;
        // 用显式全零参数，不要依赖默认值
        req->params_json =
            R"({"lineVelocityX":0.0,"lineVelocityY":0.0,"velocity":0.0})";

        state_ = State::kStarting;
        RCLCPP_INFO(get_logger(), "请求取权并启动动作 '%s'（遥控器必须已断开）",
                    start_action_.c_str());

        start_cli_->async_send_request(
            req, [this](rclcpp::Client<uniubi_motion_bridge::srv::StartMotionAction>::
                            SharedFuture future) {
                auto resp = future.get();
                if (!resp->success) {
                    start_attempts_++;
                    RCLCPP_ERROR(get_logger(),
                                 "启动动作失败(第 %d 次): error_code=%d %s",
                                 start_attempts_, resp->error_code,
                                 resp->message.c_str());
                    if (start_attempts_ < kMaxStartAttempts) {
                        RCLCPP_WARN(get_logger(),
                                    "2 秒后重试（若持续失败，检查遥控器是否已断开）");
                        state_ = State::kWaiting;
                        retry_after_ = now() + rclcpp::Duration::from_seconds(2.0);
                        return;
                    }
                    finish("启动动作失败");
                    return;
                }
                RCLCPP_INFO(get_logger(), "动作已启动，开始走向航点 %zu/%zu",
                            index_ + 1, waypoints_.size());
                state_ = State::kDriving;
                start_time_ = now();
                // 刚进入 Walk 时 /odom 可能还没上线，给一个宽限窗口，
                // 期间用 (0,0,0) 当作初始位姿（设备端会在进入 Walk 时重置原点）。
                odom_grace_deadline_ =
                    now() + rclcpp::Duration::from_seconds(odom_grace_s_);
            });
    }

    void driving() {
        // ---------- 安全条件 ----------
        if (!have_odom_) {
            // 动作刚启动、Walk 刚进入时 /odom 还没上线是正常的，
            // 宽限窗口内先用 (0,0,0) 作为初始位姿继续走；超时才判失败。
            if (now() > odom_grace_deadline_) {
                abortRun("启动 Walk 后一直没有 /odom");
                return;
            }
        } else if ((now() - last_odom_).seconds() > odom_timeout_s_) {
            abortRun("里程计超时（狗可能已不在 Walk 动作）");
            return;
        }
        if ((now() - start_time_).seconds() > max_duration_s_) {
            abortRun("超过总时长上限");
            return;
        }
        if (have_battery_ && battery_ < battery_min_) {
            abortRun("电量低于阈值");
            return;
        }
        if (have_status_ && last_error_code_ != 0) {
            abortRun("服务端报错: " + last_error_message_);
            return;
        }
        if (have_status_ &&
            (now() - start_time_).seconds() > 1.0 &&
            control_state_ != uniubi_motion_bridge::msg::MotionStatus::CONTROLLED) {
            abortRun("控制权已丢失");
            return;
        }

        // ---------- 逐航点控制 ----------
        const Waypoint& wp = waypoints_[index_];
        const double dx = wp.x - pose_x_;
        const double dy = wp.y - pose_y_;
        const double rho = std::hypot(dx, dy);

        geometry_msgs::msg::Twist cmd;  // 默认全零

        if (rho > pos_tol_) {
            aligning_ = false;
            const double target_yaw = std::atan2(dy, dx);
            const double alpha = normalizeAngle(target_yaw - pose_yaw_);

            double v = k_rho_ * rho;
            // 目标在侧后方时先把头转过去，避免"边走边画大圈"
            if (std::fabs(alpha) > 1.0) v = 0.0;
            cmd.linear.x = clamp(v, -v_max_, v_max_);
            cmd.angular.z = clamp(k_alpha_ * alpha, -w_max_, w_max_);
        } else if (wp.has_yaw) {
            // 位置到了，若该点指定朝向则原地转正
            aligning_ = true;
            const double alpha = normalizeAngle(wp.yaw - pose_yaw_);
            if (std::fabs(alpha) > yaw_tol_) {
                cmd.angular.z = clamp(k_alpha_ * alpha, -w_max_, w_max_);
            } else {
                advanceWaypoint();
            }
        } else {
            advanceWaypoint();
        }

        cmd_pub_->publish(cmd);
    }

    void advanceWaypoint() {
        cmd_pub_->publish(geometry_msgs::msg::Twist());  // 先停车
        index_++;
        if (index_ < waypoints_.size()) {
            RCLCPP_INFO(get_logger(), "到达航点 %zu，前往 %zu/%zu (%.2f, %.2f)",
                        index_, index_ + 1, waypoints_.size(),
                        waypoints_[index_].x, waypoints_[index_].y);
            return;
        }
        if (loop_) {
            index_ = 0;
            RCLCPP_INFO(get_logger(), "路径走完一轮，循环回到起点");
            return;
        }
        RCLCPP_INFO(get_logger(), "全部航点完成");
        finish("完成路径");
    }

    void abortRun(const std::string& reason) {
        RCLCPP_WARN(get_logger(), "中止: %s", reason.c_str());
        finish(reason);
    }

    /// 统一的收尾：停车 -> stop_action -> release_control -> 退出
    void finish(const std::string& reason) {
        if (state_ == State::kFinishing || state_ == State::kDone) return;
        state_ = State::kFinishing;

        cmd_pub_->publish(geometry_msgs::msg::Twist());
        RCLCPP_INFO(get_logger(), "收尾中（%s）", reason.c_str());

        if (!stop_cli_->service_is_ready()) {
            releaseThenExit();
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        stop_cli_->async_send_request(
            req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                (void)future;
                RCLCPP_INFO(get_logger(), "stop_action 已下发");
                releaseThenExit();
            });
    }

    void releaseThenExit() {
        if (!release_cli_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "release_control 服务不可用，靠租约超时兜底");
            state_ = State::kDone;
            rclcpp::shutdown();
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        release_cli_->async_send_request(
            req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                (void)future;
                RCLCPP_INFO(get_logger(), "已释放控制权，退出");
                state_ = State::kDone;
                rclcpp::shutdown();
            });
    }

    // ---------- 成员 ----------
    std::string path_file_;
    std::string start_action_;
    std::vector<Waypoint> waypoints_;
    size_t index_ = 0;

    double v_max_ = 0.25;
    double w_max_ = 0.6;
    double k_rho_ = 0.8;
    double k_alpha_ = 1.5;
    double pos_tol_ = 0.15;
    double yaw_tol_ = 0.15;
    double max_duration_s_ = 120.0;
    double battery_min_ = 0.15;
    double odom_timeout_s_ = 1.0;
    double control_rate_hz_ = 10.0;
    bool loop_ = false;

    /// 启动动作后等待首帧 /odom 的宽限时间
    double odom_grace_s_ = 15.0;
    rclcpp::Time odom_grace_deadline_{0, 0, RCL_ROS_TIME};

    State state_ = State::kWaiting;
    bool aligning_ = false;

    /// 首次取权时 bridge 要先把电机主控切回小脑再申请会话，偶发超时，
    /// 所以失败后退避重试几次，而不是直接放弃。
    static constexpr int kMaxStartAttempts = 3;
    int start_attempts_ = 0;
    rclcpp::Time retry_after_{0, 0, RCL_ROS_TIME};

    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    double pose_yaw_ = 0.0;
    bool have_odom_ = false;

    double battery_ = 1.0;
    bool have_battery_ = false;

    uint8_t control_state_ = 0;
    int32_t last_error_code_ = 0;
    std::string last_error_message_;
    bool have_status_ = false;

    rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<uniubi_motion_bridge::msg::MotionStatus>::SharedPtr status_sub_;
    rclcpp::Client<uniubi_motion_bridge::srv::StartMotionAction>::SharedPtr start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_cli_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<WaypointFollower>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("waypoint_follower"), "异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
