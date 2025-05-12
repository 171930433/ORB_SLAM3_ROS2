#ifndef IMU_RGBD_SLAM_NODE_HPP_
#define IMU_RGBD_SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "orb_slam3/System.h"
#include "utility.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class ImuRgbdSlamNode : public rclcpp::Node
{
public:
    ImuRgbdSlamNode(ORB_SLAM3::System* pSLAM);
    ~ImuRgbdSlamNode();

private:
    void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);
    void GrabImu(const ImuMsg::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;
    
    // Subscribers
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_sub;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> approximate_sync_policy;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    // Image message pointers
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    // IMU data buffer
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    std::mutex mBufMutex;
};

#endif // IMU_RGBD_SLAM_NODE_HPP_ 