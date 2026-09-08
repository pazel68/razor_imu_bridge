import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('razor_imu_bridge'), 'config')

    return launch.LaunchDescription(
        [   launch_ros.actions.Node(
                package='razor_imu_bridge',
                executable='razor_imu_node',
                name='razor_imu',
                parameters=[{
                    'accel_x_offset': -0.155379,
                    'accel_y_offset': 0.053448,
                    'accel_z_offset': 0.030605,
                    'gyro_x_offset': -0.000822,
                    'gyro_y_offset': -0.001320,
                    'gyro_z_offset': -0.000517
                }]
            ),
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
                remappings=[('/imu/data_raw', '/imu/data_raw')]
            )
        ]
    )
