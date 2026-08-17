from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='ballbot',
            executable='ballbot_combined',
            name='ballbot_combined_node',
            output='screen',
            parameters=[{
                'publish_compat_topics': True,
            }],
        ),

        # Node(
        #     package='ballbot',
        #     executable='encoder_read',
        #     name='encoder_read_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='encoder_raw',
        #     name='encoder_raw_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='engine_PWM',
        #     name='engine_pwm_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='imu_raw',
        #     name='imu_raw_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='imu_kalman',
        #     name='imu_kalman_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='IMU_to_degrees',
        #     name='imu_to_degrees_node',
        #     output='screen'
        # ),
        
        # Node(
        #     package='ballbot',
        #     executable='imu_to_quaternion',
        #     name='imu_to_quaternion_node',
        #     output='screen'
        # ),

        # Node(
        #     package='imu_complementary_filter',
        #     executable='complementary_filter_node',
        #     name='imu_filter_node',
        #     output='screen',
        #     parameters=[{
        #         'use_mag': False,
        #         'do_bias_estimation': True,
        #         'do_adaptive_gain': False,
        #         'gain_acc': 0.05,
        #         'gain_mag': 0.0,
        #         'publish_tf': False,
        #     }],
        #     remappings=[
        #         ('imu/data_raw', '/imu/data_raw'),
        #         ('imu/data', '/imu/data'),
        #     ],
        # ),

        # Node(
        #     package='ballbot',
        #     executable='motor_PWM_control',
        #     name='motor_pwm_control_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='velocity_controller',
        #     name='velocity_controller_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='main_controller',
        #     name='main_controller_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='main_controller_PID',
        #     name='main_controller_PID_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='main_controller_PID_quat',
        #     name='main_controller_PID_quat_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='imu_kalman_plot',
        #     name='imu_kalman_plot_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='check_gyro',
        #     name='check_gyro_node',
        #     output='screen'
        # ),

        # Node(
        #     package='ballbot',
        #     executable='imu_kalman_lib',
        #     name='imu_kalman_lib_node',
        #     output='screen'
        # ),

    ])