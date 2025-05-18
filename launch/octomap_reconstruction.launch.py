from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='orbslam3_ros2',
            executable='octomap_reconstruction_node',
            name='octomap_reconstruction_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'octomap_resolution': 0.05,
                'max_depth': 10.0,
                'min_depth': 0.1,
                'prob_hit': 0.7,
                'prob_miss': 0.4,
                'clamping_thres_min': 0.12,
                'clamping_thres_max': 0.97
            }],
            remappings=[
                ('camera/rgb', '/camera/rgb/image_raw'),
                ('camera/depth', '/camera/depth/image_raw')
            ]
        )
    ]) 