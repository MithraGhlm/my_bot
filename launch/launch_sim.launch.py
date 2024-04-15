import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

#ARGUMENTS = []
#for pose_element in ['x', 'y', 'z', 'yaw']:
#    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
#                     description=f'{pose_element} component of the robot pose.'))

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    # Path to the URDF file
    urdf_file = get_package_share_directory('my_bot') + '/description/robot.urdf'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(package_name),'launch/rsp.launch.py'
                )), launch_arguments={'use_sim_time': 'true'}.items()
    )


    # Parameters
    #x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    #yaw = LaunchConfiguration('yaw')
    #z_robot = OffsetParser(z, -0.005)



    # Launch Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_ign_gazebo'), '/launch', '/ign_gazebo.launch.py'
        ])
    )

#    ExecuteProcess(
#            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
#            output='screen'),

    # Spawn the robot in Ignition Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            "-file", urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.07',
            '-Y', '0' #yaw
        ]
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        ign_gazebo,
        spawn_robot
    ])
