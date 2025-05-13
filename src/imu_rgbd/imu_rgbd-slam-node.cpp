#include "imu_rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

ImuRgbdSlamNode::ImuRgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("orbslam3_imu_rgbd"),
    m_SLAM(pSLAM)
{
    // 声明参数
    this->declare_parameter("rgb_topic", "/world/world_demo/model/tugbot/link/camera_front/sensor/color/image");
    this->declare_parameter("depth_topic", "/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image");
    this->declare_parameter("imu_topic", "/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu");

    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "rgb_topic: %s", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic.c_str());

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, rgb_topic);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, depth_topic);
    imu_sub = this->create_subscription<ImuMsg>(imu_topic, 1000, std::bind(&ImuRgbdSlamNode::GrabImu, this, _1));
    
    // 初始化TF发布器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 初始化轨迹发布器
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orbslam3/trajectory", 10);
    path_msg_.header.frame_id = "world/world_demo";

    // Synchronizer
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&ImuRgbdSlamNode::GrabRGBD, this);
}

ImuRgbdSlamNode::~ImuRgbdSlamNode()
{
    RCLCPP_INFO(this->get_logger(), "~ImuRgbdSlamNode done");

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void ImuRgbdSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mBufMutex);
    vImuMeas.push_back(ORB_SLAM3::IMU::Point(msg->linear_acceleration.x,
                                            msg->linear_acceleration.y,
                                            msg->linear_acceleration.z,
                                            msg->angular_velocity.x,
                                            msg->angular_velocity.y,
                                            msg->angular_velocity.z,
                                            Utility::StampToSec(msg->header.stamp)));
}

void ImuRgbdSlamNode::PublishTF(const Sophus::SE3f& Twc, const rclcpp::Time& stamp)
{
    if(Twc.matrix().isZero())
        return;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "world/world_demo";
    transform.child_frame_id = "model/tugbot/link/camera_front";

    // 将Sophus::SE3f转换为Eigen::Isometry3d
    Eigen::Isometry3d T_eigen (Twc.cast<double>().matrix());
    // 转换为ROS消息
    transform.transform = tf2::eigenToTransform(T_eigen).transform;

    // 发布TF
    tf_broadcaster_->sendTransform(transform);

    // 发布轨迹
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose);
    path_pub_->publish(path_msg_);
}

void ImuRgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Get IMU measurements
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        vImuMeas = this->vImuMeas;
        this->vImuMeas.clear();
    }

    // 跟踪相机位姿
    Sophus::SE3f Tc_c0 = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp), vImuMeas);
    
    // 绕world系的X轴转-90°
    static Sophus::SE3f Tw_c0 = Sophus::SE3f(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()).toRotationMatrix(), Eigen::Vector3f::Zero());
    Sophus::SE3f Tw_c = Tw_c0 * Tc_c0.inverse();


    // 发布TF
    PublishTF(Tw_c, msgRGB->header.stamp);
} 