"""Launch the physical R10 inventory and meter without the abstract launcher controller."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('pingpong_launcher_sim')
    prefix = get_package_prefix('pingpong_launcher_sim')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    world = LaunchConfiguration('world').perform(context)
    cmd = ['ign', 'gazebo', '-r', '-v', '3']
    if not gui:
        cmd.append('-s')
    cmd.append(world)
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            os.path.join(share, 'models'), os.environ.get('IGN_GAZEBO_RESOURCE_PATH')]))),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.pathsep.join(filter(None, [
            os.path.join(prefix, 'lib'), os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH')]))),
        ExecuteProcess(cmd=cmd, output='screen', name='gazebo'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'physical_bridge.yaml')}],
             output='screen'),
]



def generate_launch_description():
    share = get_package_share_directory('pingpong_launcher_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'physical_100.sdf')),
        OpaqueFunction(function=setup)])
