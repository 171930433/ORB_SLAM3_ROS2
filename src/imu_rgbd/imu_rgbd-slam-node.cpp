#include "imu_rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

ImuRgbdSlamNode::ImuRgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    // RGBD subscribers
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(shared_ptr<rclcpp::Node>(this), "camera/depth");

    // IMU subscriber
    imu_sub = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&ImuRgbdSlamNode::GrabImu, this, _1));

    // Synchronizer
    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&ImuRgbdSlamNode::GrabRGBD, this);
}

ImuRgbdSlamNode::~ImuRgbdSlamNode()
{
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

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp), vImuMeas);
} 