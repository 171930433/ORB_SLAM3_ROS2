#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::chrono_literals;

class OctomapReconstructionNode : public rclcpp::Node
{
public:
    OctomapReconstructionNode() : Node("octomap_reconstruction_node")
    {
        // 声明参数
        this->declare_parameter("octomap_resolution", 0.05);
        this->declare_parameter("max_depth", 10.0);
        this->declare_parameter("min_depth", 0.1);
        this->declare_parameter("prob_hit", 0.7);
        this->declare_parameter("prob_miss", 0.4);
        this->declare_parameter("clamping_thres_min", 0.12);
        this->declare_parameter("clamping_thres_max", 0.97);

        // 获取参数
        double resolution = this->get_parameter("octomap_resolution").as_double();
        max_depth_ = this->get_parameter("max_depth").as_double();
        min_depth_ = this->get_parameter("min_depth").as_double();
        double prob_hit = this->get_parameter("prob_hit").as_double();
        double prob_miss = this->get_parameter("prob_miss").as_double();
        double clamping_thres_min = this->get_parameter("clamping_thres_min").as_double();
        double clamping_thres_max = this->get_parameter("clamping_thres_max").as_double();

        // 创建订阅者
        rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/rgb");
        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, "camera/depth");

        // 创建同步器
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_policy;
        sync_ = std::make_shared<message_filters::Synchronizer<sync_policy>>(
            sync_policy(10), *rgb_sub_, *depth_sub_);
        
        // 使用std::bind注册回调
        sync_->registerCallback(&OctomapReconstructionNode::rgbd_callback, this);

        // 创建发布者
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        // 创建TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化OctoMap
        octree_ = std::make_shared<octomap::OcTree>(resolution);
        octree_->setProbHit(prob_hit);
        octree_->setProbMiss(prob_miss);
        octree_->setClampingThresMin(clamping_thres_min);
        octree_->setClampingThresMax(clamping_thres_max);

        RCLCPP_INFO(this->get_logger(), "Octomap reconstruction node initialized");
    }

private:
    void rgbd_callback(const sensor_msgs::msg::Image::SharedPtr rgb_msg,
                      const sensor_msgs::msg::Image::SharedPtr depth_msg)
    {
        try
        {
            // 获取相机内参
            double fx = 615.9603271484375;  // 从配置文件中获取
            double fy = 616.227294921875;
            double cx = 419.83026123046875;
            double cy = 245.1431427001953;

            // 转换图像
            cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(rgb_msg);
            cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

            // 创建点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud->width = depth_ptr->image.cols;
            cloud->height = depth_ptr->image.rows;
            cloud->is_dense = false;
            cloud->points.resize(cloud->width * cloud->height);

            // 获取相机位姿
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    "map", "camera_link",
                    tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                return;
            }

            // 转换深度图像到点云
            for (int v = 0; v < depth_ptr->image.rows; v++) {
                for (int u = 0; u < depth_ptr->image.cols; u++) {
                    float depth = depth_ptr->image.at<float>(v, u);
                    if (depth > min_depth_ && depth < max_depth_) {  // 使用参数化的深度范围
                        pcl::PointXYZRGB& point = cloud->points[v * depth_ptr->image.cols + u];
                        point.x = (u - cx) * depth / fx;
                        point.y = (v - cy) * depth / fy;
                        point.z = depth;
                        point.r = rgb_ptr->image.at<cv::Vec3b>(v, u)[0];
                        point.g = rgb_ptr->image.at<cv::Vec3b>(v, u)[1];
                        point.b = rgb_ptr->image.at<cv::Vec3b>(v, u)[2];
                    }
                }
            }

            // 发布点云
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header = rgb_msg->header;
            cloud_pub_->publish(cloud_msg);

            // 更新OctoMap
            for (const auto& point : cloud->points) {
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    octomap::point3d endpoint(point.x, point.y, point.z);
                    octree_->updateNode(endpoint, true);
                }
            }

            // 发布OctoMap
            octomap_msgs::msg::Octomap octomap_msg;
            octomap_msgs::binaryMapToMsg(*octree_, octomap_msg);
            octomap_msg.header = rgb_msg->header;
            octomap_pub_->publish(octomap_msg);

        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGBD data: %s", e.what());
        }
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::shared_ptr<octomap::OcTree> octree_;
    double max_depth_;
    double min_depth_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapReconstructionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 