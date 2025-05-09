#include <rclcpp/rclcpp.hpp>
#include "imu_rgbd-slam-node.hpp"
#include "System.h"
#include <string>
#include <chrono>
#include <memory>

using namespace std;

int main(int argc, char **argv)
{
    RCLCPP_ERROR(rclcpp::get_logger("orbslam3_ros2"), "argc is: %d", argc);
    // for (int i = 0; i < argc; i++)
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("orbslam3_ros2"), "argv[%d] is: %s", i, argv[i]);
    // }

    if (argc < 3)
    {
        cerr << endl
             << "Usage: ros2 run orbslam3 imu_rgbd path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    RCLCPP_ERROR(rclcpp::get_logger("orbslam3_ros2"), "argv[%d] is: %s", 1, argv[1]);
    RCLCPP_ERROR(rclcpp::get_logger("orbslam3_ros2"), "argv[%d] is: %s", 2, argv[2]);
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    auto node = std::make_shared<ImuRgbdSlamNode>(&SLAM);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}