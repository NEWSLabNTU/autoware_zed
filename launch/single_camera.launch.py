from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package name
    pkg_name = 'autoware_zed'
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    input_topic = LaunchConfiguration('input_topic', default='/zed/zed_node/obj_det/objects')
    output_topic = LaunchConfiguration('output_topic', default='/perception/object_recognition/objects')
    
    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'autoware_zed.yaml'
    ])
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_input_topic_cmd = DeclareLaunchArgument(
        'input_topic',
        default_value='/zed/zed_node/obj_det/objects',
        description='Input topic with ZED objects'
    )
    
    declare_output_topic_cmd = DeclareLaunchArgument(
        'output_topic',
        default_value='/perception/object_recognition/objects',
        description='Output topic for Autoware objects'
    )
    
    # Transformer node
    transformer_node = Node(
        package=pkg_name,
        executable='autoware_zed_node',
        name='autoware_zed',
        namespace='',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': input_topic,
                'output_topic': output_topic
            }
        ],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_input_topic_cmd)
    ld.add_action(declare_output_topic_cmd)
    
    # Add node
    ld.add_action(transformer_node)
    
    return ld