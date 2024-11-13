from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for publish frequency and log level
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='500',
        description='Frequency (in ms) at which the publisher publishes messages.'
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

    return LaunchDescription([
        publish_frequency_arg,
        publisher_log_level_arg,
        subscriber_log_level_arg,
        talker_node,
        listener_node
    ])
