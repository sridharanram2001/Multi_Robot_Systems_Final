import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    gazebo_ros_launch_path = os.path.join(get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py")
    world_path = os.path.join(get_package_share_path("mrs_gazebo_sim"), "worlds", "empty.world")
    
    package_name='mrs_gazebo_sim' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_ros_launch_path), 
        launch_arguments={ 'world': world_path}.items())
    
    #gazebo = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
     #               get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
      #      )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot','-x','4.0','-y','-4.0','-z','0.5'],
                        output='screen')

    #diff_drive_spawner = Node(package='controller_manager', executable='spawner',
     #                   arguments=['diff_cont'])

    #joint_broad_spawner = Node(package='controller_manager', executable='spawner',
     #                   arguments=['joint_broad'])

    #arm_control = Node(package='controller_manager', executable='spawner',
     #                   arguments=['arm_controller'])




    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,])
        #diff_drive_spawner,
        #joint_broad_spawner,
        #arm_control,
    #])
