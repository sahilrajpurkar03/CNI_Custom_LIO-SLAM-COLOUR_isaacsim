#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

using std::placeholders::_1;

// Custom point type with ring and time fields
struct PointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (float, timestamp, timestamp)
)

class PointCloudConverter : public rclcpp::Node
{
public:
  PointCloudConverter() : Node("rs_converter")
  {
    // Declare parameters
    this->declare_parameter<std::string>("frame_id", "lidar_link");
    this->declare_parameter<int>("N_SCAN", 32);  // Typical for 16-beam LiDAR
    this->declare_parameter<float>("fov_bottom", -60.0);
    this->declare_parameter<float>("fov_top", 60.0);
    this->declare_parameter<float>("min_dist", 0.1);
    this->declare_parameter<float>("max_dist", 60.0);
    this->declare_parameter<int>("publish_every_nth", 1);
    
    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    N_SCAN_ = this->get_parameter("N_SCAN").as_int();
    fov_bottom_ = this->get_parameter("fov_bottom").as_double();
    fov_top_ = this->get_parameter("fov_top").as_double();
    min_dist_ = this->get_parameter("min_dist").as_double();
    max_dist_ = this->get_parameter("max_dist").as_double();
    publish_every_nth_ = this->get_parameter("publish_every_nth").as_int();
    
    // Calculate vertical resolution
    ang_res_y_ = (fov_top_ - fov_bottom_) / (N_SCAN_ - 1);

    rclcpp::QoS qos(rclcpp::KeepLast(50));
    qos.best_effort();
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/point_cloud", qos, std::bind(&PointCloudConverter::lidar_callback, this, _1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", qos);

    RCLCPP_INFO(this->get_logger(), 
               "Initialized PointCloudConverter with %d vertical scans (%.1f° to %.1f°)", 
               N_SCAN_, fov_bottom_, fov_top_);
  }

private:
  void publish_cloud(pcl::PointCloud<PointXYZIRT>::Ptr cloud, 
                    const std_msgs::msg::Header& header)
  {
    cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = header;
    output_msg.header.frame_id = frame_id_;
    pub_->publish(output_msg);
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (++counter_ % publish_every_nth_ != 0)
      return;

    // Check for required fields
    bool has_intensity = false;
    for (const auto &field : msg->fields) {
      if (field.name == "intensity") {
        has_intensity = true;
        break;
      }
    }

    // Convert to PCL format
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pc(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (has_intensity) {
      pcl::fromROSMsg(*msg, *input_pc);
    } else {
      // Fall back to XYZ and calculate intensity based on distance
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_pc(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *xyz_pc);
      input_pc->points.resize(xyz_pc->points.size());
      for (size_t i = 0; i < xyz_pc->points.size(); ++i) {
        input_pc->points[i].x = xyz_pc->points[i].x;
        input_pc->points[i].y = xyz_pc->points[i].y;
        input_pc->points[i].z = xyz_pc->points[i].z;
        float distance = sqrt(xyz_pc->points[i].x * xyz_pc->points[i].x +
                             xyz_pc->points[i].y * xyz_pc->points[i].y +
                             xyz_pc->points[i].z * xyz_pc->points[i].z);
        input_pc->points[i].intensity = std::min(255.0f, distance * 10.0f);
      }
    }

    if (input_pc->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
      return;
    }

    pcl::PointCloud<PointXYZIRT>::Ptr output_pc(new pcl::PointCloud<PointXYZIRT>());
    output_pc->reserve(input_pc->points.size());

    // Process each point
    for (size_t i = 0; i < input_pc->points.size(); ++i) {
      const auto &pt = input_pc->points[i];
      
      // Calculate distance for filtering
      float distance = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (distance < min_dist_ || distance > max_dist_) {
        continue;
      }

      PointXYZIRT new_point;
      new_point.x = pt.x;
      new_point.y = pt.y;
      new_point.z = pt.z;
      new_point.intensity = pt.intensity;
      
      // Calculate vertical angle and ring number
      float vertical_angle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180.0 / M_PI;
      float row_idn = (vertical_angle - fov_bottom_) / ang_res_y_;
      
      if (row_idn >= 0 && row_idn < N_SCAN_) {
        new_point.ring = static_cast<uint16_t>(row_idn);
      } else {
        new_point.ring = 0;  // Default if outside expected range
      }
      
      // Calculate relative timestamp (0-1 over the scan)
      new_point.timestamp = static_cast<float>(i) / input_pc->points.size();
      
      output_pc->points.push_back(new_point);
    }

    // Publish the converted cloud
    publish_cloud(output_pc, msg->header);

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    RCLCPP_DEBUG(this->get_logger(), 
                "Published %zu points (original: %zu) at %.3f (intensity: %s)",
                output_pc->size(), 
                input_pc->size(),
                timestamp, 
                has_intensity ? "yes" : "calculated");
  }

  // Parameters
  std::string frame_id_;
  int N_SCAN_;
  float fov_bottom_;
  float fov_top_;
  float min_dist_;
  float max_dist_;
  float ang_res_y_;
  int publish_every_nth_;
  
  // State
  int counter_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudConverter>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}