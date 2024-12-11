from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    my_robot_package_path = FindPackageShare('my_robot')
    urdf_path = 'robot.urdf.xacro'
    rviz_config = PathJoinSubstitution([my_robot_package_path, 'rviz', 'urdf.rviz'])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    return LaunchDescription([
        Node(package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
