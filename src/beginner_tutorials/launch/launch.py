from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    freq_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='2.0',
        description='Frequency for the publisher node in Hz'
    )
    record_bag_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='false', 
        description='Enable or disable ros bag recording'
    )

    publisher_log_level_arg = DeclareLaunchArgument(
        'publisher_log_level',
        default_value='DEBUG',
        description='Logging level for the Publisher node.'
    )

    subscriber_log_level_arg = DeclareLaunchArgument(
        'subscriber_log_level',
        default_value='INFO',
        description='Logging level for the Subscriber node.'
    )

    # Configure the Publisher (talker) node with the specified log level
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='publisher_service_node',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('publisher_log_level')]
    )

    # Configure the Subscriber (listener) node with the specified log level
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('subscriber_log_level')]
    )
        # create handle for conditional recording of ros bag
    def conditional_rosbag_record(context):
        if LaunchConfiguration('enable_recording').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '--all'],
                    output='screen',
                    cwd='src/beginner_tutorials/results'  # Set the working directory
                )
            ]
        return []

    return LaunchDescription([
        publisher_log_level_arg,
        freq_arg,
        record_bag_arg,
        subscriber_log_level_arg,
        talker_node,
        listener_node,
        OpaqueFunction(function=conditional_rosbag_record)
    ])
