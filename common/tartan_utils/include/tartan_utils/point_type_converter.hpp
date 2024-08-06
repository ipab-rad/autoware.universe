#ifndef POINT_TYPE_CONVERTER_HPP
#define POINT_TYPE_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace tartan_utils {

// Autoware point struct
struct PointXYZIRCAEDT
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  std::uint32_t time_stamp;
};

class PointTypeConverter : public rclcpp::Node
{
public:
  PointTypeConverter(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

} // namespace tartan_utils

#endif // POINT_TYPE_CONVERTER_HPP
