import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

print(os.path.realpath(__file__))

ld = LaunchDescription()

def generate_launch_description():
    # Configuration
    world = LaunchConfiguration('world')
    print('world =', world)
    world_file_name = 'car_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'), 'worlds', world_file_name)
    print('world file name = %s' % world)

    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    gazebo_model_path = os.path.join(get_package_share_directory('ros2_term_project'), 'models')
    gazebo_env = {'GAZEBO_MODEL_PATH': gazebo_model_path, 'LD_LIBRARY_PATH': os.environ['LD_LIBRARY_PATH']}
    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen',
        additional_env=gazebo_env,
    )

    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)

    prius_hybrid_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'prius_hybrid',
            '-file', '/home/ros2/AutoCar/Ros-Project/ros2_term_project/models/prius_hybrid/model.sdf'
        ],
        output='screen',
    )
    
    my_controller_node = Node(
        package='ros2_term_project',
        executable='my_controller_node',
        name='my_controller_node',
        output='screen',
    )
    
    my_subscriber_node = Node(
        package='ros2_term_project',
        executable='my_subscriber_node',
        name='my_subscriber_node',
        output='screen',
    )
    

    ld.add_action(prius_hybrid_spawn)
    ld.add_action(my_controller_node)
    ld.add_action(my_subscriber_node)

    return ld

