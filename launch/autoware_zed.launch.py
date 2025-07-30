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
    
    # For multi-camera setup
    front_camera_ns = LaunchConfiguration('front_camera_ns', default='/zed_multi/zed_front')
    back_camera_ns = LaunchConfiguration('back_camera_ns', default='/zed_multi/zed_back')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_front_camera_ns_cmd = DeclareLaunchArgument(
        'front_camera_ns',
        default_value='/zed_multi/zed_front',
        description='Namespace for front camera'
    )
    
    declare_back_camera_ns_cmd = DeclareLaunchArgument(
        'back_camera_ns',
        default_value='/zed_multi/zed_back',
        description='Namespace for back camera'
    )
    
    # Transformer node for front camera
    front_transformer_node = Node(
        package=pkg_name,
        executable='autoware_zed_node',
        name='autoware_zed_front',
        namespace='',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': [front_camera_ns, '/obj_det/objects'],
                'output_topic': '/perception/object_recognition/objects/front'
            }
        ],
        output='screen',
        remappings=[]
    )
    
    # Transformer node for back camera
    back_transformer_node = Node(
        package=pkg_name,
        executable='autoware_zed_node',
        name='autoware_zed_back',
        namespace='',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': [back_camera_ns, '/obj_det/objects'],
                'output_topic': '/perception/object_recognition/objects/back'
            }
        ],
        output='screen',
        remappings=[]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_front_camera_ns_cmd)
    ld.add_action(declare_back_camera_ns_cmd)
    
    # Add nodes
    ld.add_action(front_transformer_node)
    ld.add_action(back_transformer_node)
    
    return ld
