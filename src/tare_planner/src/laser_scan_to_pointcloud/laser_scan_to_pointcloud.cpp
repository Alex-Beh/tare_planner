#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/filters/passthrough.h>

using std::placeholders::_1;

class LaserScanSubscriber : public rclcpp::Node
{
public:
  LaserScanSubscriber()
      : Node("laser_scan_to_pointcloud")
  {
  //   rmw_qos_profile_t qos_profile =
  //       {
  //           RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  //           10,
  //           RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  //           RMW_QOS_POLICY_DURABILITY_VOLATILE,
  //           RMW_QOS_DEADLINE_DEFAULT,
  //           RMW_QOS_LIFESPAN_DEFAULT,
  //           RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  //           RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  //           false};
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos_profile, std::bind(&LaserScanSubscriber::scan_callback, this, _1));
    stacked_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 1);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr stacked_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
    // Populate the stacked point cloud
    for (int layer = 0; layer < 16; ++layer)
    {
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {

          if(scan_msg->ranges[i]<scan_msg->range_min || 
              scan_msg->ranges[i]>scan_msg->range_max){
            continue;
          }

          pcl::_PointXYZI point;
          double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
          // TODO: cache the cos/sin values
          point.x = scan_msg->ranges[i] * cos(angle);
          point.y = scan_msg->ranges[i] * sin(angle);
          point.z = -0.5 + layer * 0.05;  // Adjust the z-offset as needed
          point.intensity = 1.0;
          stacked_cloud->points.push_back(point);
       }
    }
    
    stacked_cloud->width = stacked_cloud->points.size()/16;  // Width remains the same
    stacked_cloud->height = 16;  // Height is set to the number of layers

    // Convert to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_pointcloud_msg;
    pcl::toROSMsg(*stacked_cloud, output_pointcloud_msg);
    output_pointcloud_msg.header = scan_msg->header;
    
    // Publish the stacked point cloud
    stacked_pc_publisher_->publish(output_pointcloud_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stacked_pc_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}