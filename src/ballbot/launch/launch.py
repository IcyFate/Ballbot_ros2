from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ballbot',
            executable='encoder_read',
            name='encoder_read_node',
            output='screen'
        ),

        # Node(
        #     package='ballbot',
        #     executable='encoder_raw',
        #     name='encoder_raw_node',
        #     output='screen'
        # ),

        Node(
            package='ballbot',
            executable='engine_PWM',
            name='engine_pwm_node',
            output='screen'
        ),

        # Node(
        #     package='ballbot',
        #     executable='imu_raw',
        #     name='imu_raw_node',
        #     output='screen'
        # ),

        Node(
            package='ballbot',
            executable='imu_kalman',
            name='imu_kalman_node',
            output='screen'
        ),

        # Node(
        #     package='ballbot',
        #     executable='imu_kalman_plot',
        #     name='imu_kalman_plot_node',
        #     output='screen'
        # ),

    ])