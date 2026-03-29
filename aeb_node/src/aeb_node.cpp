#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

class AEBNode : public rclcpp::Node
{
public:
  AEBNode() : Node("aeb_node")
  {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/pointcloud_raw_ex", 10,
      std::bind(&AEBNode::lidarCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 10,
      std::bind(&AEBNode::odomCallback, this, std::placeholders::_1));

    distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/aeb/distance", 10);

    RCLCPP_INFO(this->get_logger(), "AEB Node started — waiting for LiDAR...");
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Got pointcloud: %d points, frame: %s",
      msg->width * msg->height,
      msg->header.frame_id.c_str());

    auto out = std_msgs::msg::Float32();
    out.data = 99.0;
    distance_pub_->publish(out);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    velocity_ = msg->twist.twist.linear.x;
    RCLCPP_DEBUG(this->get_logger(), "Velocity: %.2f m/s", velocity_);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  double velocity_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBNode>());
  rclcpp::shutdown();
  return 0;
}
