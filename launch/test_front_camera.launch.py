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
    
    # Transformer node for front camera with correct topic
    front_transformer_node = Node(
        package=pkg_name,
        executable='autoware_zed_node',
        name='autoware_zed_front_test',
        namespace='',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': '/zed_multi/zed_front/obj_det/objects',
                'output_topic': '/perception/object_recognition/objects/front_test'
            }
        ],
        output='screen',
        remappings=[]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(front_transformer_node)
    
    return ld