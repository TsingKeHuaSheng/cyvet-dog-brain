#include <cmath>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace office4_route {
class DrawRoute : public rviz_common::Tool {
  using Path = nav_msgs::msg::Path;
  rviz_rendering::ViewportProjectionFinder projection_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Path>::SharedPtr preview_, completed_;
  Path path_;
  bool drawing_ = false;

public:
  void onInitialize() override {
    setName("Draw Route");
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    preview_ = node_->create_publisher<Path>("/line_route/stroke", 1);
    completed_ = node_->create_publisher<Path>("/line_route/drawn", 1);
  }
  void activate() override {setStatus("Drag from A to B. Release to save.");}
  void deactivate() override {
    drawing_ = false;
    path_.poses.clear();
    path_.header.frame_id = "map";
    preview_->publish(path_);
  }
  int processMouseEvent(rviz_common::ViewportMouseEvent & e) override {
    if (context_->getFixedFrame() != "map") {
      setStatus("Select map as Fixed Frame."); return 0;
    }
    if (e.leftDown()) {
      drawing_ = true;
      path_ = Path();
      path_.header.frame_id = "map";
    }
    if (!drawing_) return 0;
    auto hit = projection_.getViewportPointProjectionOnXYPlane(
      e.panel->getRenderWindow(), e.x, e.y);
    if (hit.first) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = hit.second.x; p.pose.position.y = hit.second.y;
      p.pose.orientation.w = 1.0;
      const auto & q = path_.poses.empty() ? p.pose.position : path_.poses.back().pose.position;
      double d = std::hypot(p.pose.position.x-q.x, p.pose.position.y-q.y);
      if (path_.poses.empty() || d >= 0.025 || (e.leftUp() && d > 0.001)) {
        path_.poses.push_back(p);
        path_.header.stamp = node_->now();
        preview_->publish(path_);
      }
    }
    if (e.leftUp()) {
      if (hit.first && path_.poses.size() >= 2) completed_->publish(path_);
      deactivate();
      return Render | Finished;
    }
    return Render;
  }
};
}
PLUGINLIB_EXPORT_CLASS(office4_route::DrawRoute, rviz_common::Tool)
