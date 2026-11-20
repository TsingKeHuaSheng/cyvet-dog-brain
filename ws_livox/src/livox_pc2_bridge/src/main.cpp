#include <cstring>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using livox_msg = livox_ros_driver2::msg::CustomMsg;
using pc2_msg   = sensor_msgs::msg::PointCloud2;

class LivoxPc2Bridge : public rclcpp::Node {
public:
  LivoxPc2Bridge() : Node("livox_pc2_bridge") {
    pub_ = create_publisher<pc2_msg>("/livox/lidar_pc2", rclcpp::QoS(10));  // RELIABLE，兼容 RViz 默认
    sub_ = create_subscription<livox_msg>(
        "/livox/lidar", rclcpp::SensorDataQoS(),
        std::bind(&LivoxPc2Bridge::on_msg, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "waiting for /livox/lidar (CustomMsg)...");
  }

private:
  void on_msg(const livox_msg::SharedPtr msg) {
    const size_t n = msg->point_num;
    if (n == 0) return;

    pc2_msg out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(n);
    out.is_bigendian = false;
    out.point_step = 16;
    out.row_step = static_cast<uint32_t>(16 * n);
    out.is_dense = true;

    out.fields.resize(4);
    const char * names[4] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; ++i) {
      out.fields[i].name = names[i];
      out.fields[i].offset = static_cast<uint32_t>(4 * i);
      out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[i].count = 1;
    }

    out.data.resize(n * 16);
    float * p = reinterpret_cast<float *>(out.data.data());
    for (size_t i = 0; i < n; ++i) {
      const auto & q = msg->points[i];
      p[4 * i + 0] = q.x;
      p[4 * i + 1] = q.y;
      p[4 * i + 2] = q.z;
      p[4 * i + 3] = static_cast<float>(q.reflectivity);
    }
    pub_->publish(out);

    if (++count_ % 100 == 1) {
      RCLCPP_INFO(get_logger(), "converted %ld frames, %zu points, frame=%s",
                  count_, n, msg->header.frame_id.c_str());
    }
  }

  rclcpp::Publisher<pc2_msg>::SharedPtr pub_;
  rclcpp::Subscription<livox_msg>::SharedPtr sub_;
  long count_ = 0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPc2Bridge>());
  rclcpp::shutdown();
  return 0;
}
