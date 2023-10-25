#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/common/common.h>
// #include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>


class ESDFSplitter: public rclcpp::Node
{

  public:


  ESDFSplitter()
    : Node("esdf_splitter")
  {

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/nvblox_node/esdfAABB_pointcloud", 1, std::bind(&ESDFSplitter::esdf_callback, this, std::placeholders::_1));

    pub_obs_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/nvblox_node/esdfAABB_obs", 1);
    
    pub_unk_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/nvblox_node/esdfAABB_unk", 1);

  }


  private:

  void esdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    RCLCPP_DEBUG(this->get_logger(), "points_size (%d)",cloud->height * cloud->width);

    // define a new container for the data
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_unk(new pcl::PointCloud<pcl::PointXYZI>);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass_obs;
    pass_obs.setInputCloud (cloud);
    pass_obs.setFilterFieldName ("intensity");
    pass_obs.setFilterLimits (0.0, 1.0);
    pass_obs.filter (*cloud_obs);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass_unk;
    pass_unk.setInputCloud (cloud);
    pass_unk.setFilterFieldName ("intensity");
    pass_unk.setFilterLimits (-1.0, -0.5);
    pass_unk.filter (*cloud_unk);
    
    
    RCLCPP_DEBUG(this->get_logger(), "obs points_size (%d) ",cloud_obs->height * cloud_obs->width);
    RCLCPP_DEBUG(this->get_logger(), "unk points_size (%d) ",cloud_unk->height * cloud_unk->width);


    // publish both messages
    sensor_msgs::msg::PointCloud2 obs_msg;
    pcl::toROSMsg(*cloud_obs, obs_msg);
    pub_obs_->publish(obs_msg);
    
    sensor_msgs::msg::PointCloud2 unk_msg;
    pcl::toROSMsg(*cloud_unk, unk_msg);
    pub_unk_->publish(unk_msg);



  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obs_; 
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unk_; 

}; // class esdf splitter


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    auto node = std::make_shared<ESDFSplitter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
