from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the directories
    pkg_share = FindPackageShare(package='swerve_steering_controller').find('swerve_steering_controller')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Default configuration values
    default_model_path = os.path.join(pkg_share, 'test/urdf/robot.urdf')


    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[default_model_path]
    )

    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description', '-z', '0.2', '-package_to_model'],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
    )

    ld = LaunchDescription()
    
    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'))
    ld.add_action(DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable rqt_robot_steering'))
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'))
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo)
#    ld.add_action(spawn_entity_node)

    return ld