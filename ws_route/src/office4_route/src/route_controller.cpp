// Office4 uses Humble RPP for every velocity and footprint check. Retain a
// blocked goal at zero so that the next cycle can check the same route again.
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace office4_route
{
using Rpp = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
using Marker = visualization_msgs::msg::Marker;
using Path = nav_msgs::msg::Path;
using Clock = std::chrono::steady_clock;

// Nav2 normally checks progress before asking RPP for the next velocity.
// Reset its baseline only while fresh collision-hold reports are arriving.
class RouteProgressChecker : public nav2_controller::PoseProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name) override
  {
    PoseProgressChecker::initialize(parent, name);
    holds_ = parent.lock()->create_subscription<std_msgs::msg::Bool>(
      "footprint_blocked", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          last_hold_ms_.store(nowMs());
        }
      });
  }

  bool check(geometry_msgs::msg::PoseStamped & pose) override
  {
    if (nowMs() - last_hold_ms_.load() < 350) {
      reset();
    }
    return PoseProgressChecker::check(pose);
  }

private:
  static int64_t nowMs()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      Clock::now().time_since_epoch()).count();
  }

  std::atomic<int64_t> last_hold_ms_{-1000};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr holds_;
};

class RouteController : public Rpp
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    Rpp::configure(parent, name, tf, map);
    auto node = parent.lock();
    state_ = node->create_publisher<std_msgs::msg::Bool>("footprint_blocked", 10);
    marker_ = node->create_publisher<Marker>("footprint_contact_pose", 10);
    arc_sub_ = node->create_subscription<Path>("lookahead_collision_arc", 10,
      [this](Path::ConstSharedPtr arc) {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        arc_ = arc;
        publishProjectedMarker();
      });
  }

  void activate() override
  {
    Rpp::activate();
    state_->on_activate();
    marker_->on_activate();
  }

  void deactivate() override
  {
    Rpp::deactivate();
    state_->on_deactivate();
    marker_->on_deactivate();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override
  {
    auto now = Clock::now();
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    try {
      cmd = Rpp::computeVelocityCommands(pose, velocity, checker);
    } catch (const nav2_core::PlannerException & e) {
      if (std::string(e.what()) != "RegulatedPurePursuitController detected collision ahead!") {
        throw;
      }
      blocked_ = true;
      clear_since_.reset();
      report(true);
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> grid_lock(*costmap_->getMutex());
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if (inCollision(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation))) {
        pending_.reset();
        mark(pose.header, pose.pose.position);
      } else {
        // In Humble the last pose in *this cycle's* RPP arc is the first
        // predicted colliding robot centre. An earlier arc is unrelated.
        pending_ = pose.header;
        publishProjectedMarker();
      }
      return cmd;
    }
    if (blocked_) {
      if (!clear_since_ || now - last_valid_ > std::chrono::milliseconds(300)) {
        clear_since_ = now;
      }
      last_valid_ = now;
      if (now - *clear_since_ < std::chrono::milliseconds(500)) {
        cmd.twist = geometry_msgs::msg::Twist();
      } else {
        blocked_ = false;
        std::lock_guard<std::mutex> lock(marker_mutex_);
        pending_.reset();
        Marker m;
        m.header = pose.header;
        m.action = Marker::DELETE;
        marker_->publish(m);
      }
    }
    report(blocked_);
    return cmd;
  }

private:
  void report(bool blocked)
  {
    std_msgs::msg::Bool msg;
    msg.data = blocked;
    state_->publish(msg);
  }

  void publishProjectedMarker()
  {
    if (pending_ && arc_ && !arc_->poses.empty() && pending_->stamp == arc_->header.stamp &&
      pending_->frame_id == arc_->header.frame_id)
    {
      mark(*pending_, arc_->poses.back().pose.position);
      pending_.reset();
    }
  }

  void mark(const std_msgs::msg::Header & header, geometry_msgs::msg::Point point)
  {
    Marker m;
    m.header = header;
    m.type = Marker::SPHERE;
    m.action = Marker::ADD;
    m.pose.position = point;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.14;
    m.color.r = m.color.a = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(1.2);
    marker_->publish(m);
  }

  bool blocked_{false};
  std::optional<Clock::time_point> clear_since_;
  Clock::time_point last_valid_{};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr state_;
  rclcpp_lifecycle::LifecyclePublisher<Marker>::SharedPtr marker_;
  rclcpp::Subscription<Path>::SharedPtr arc_sub_;
  std::mutex marker_mutex_;
  Path::ConstSharedPtr arc_;
  std::optional<std_msgs::msg::Header> pending_;
};
}  // namespace office4_route
PLUGINLIB_EXPORT_CLASS(office4_route::RouteController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(office4_route::RouteProgressChecker, nav2_core::ProgressChecker)
