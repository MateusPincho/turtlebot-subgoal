'''
Este launch deve: 
- Iniciar o SLAM -> usar o launch do slam de turtlebot3_cartographer
- Iniciar o no de distancia do Lidar
- Iniciar o fuzzy planner para gerar subgoal
- Iniciar o go to goal para publicar o cmd vel
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )
    
    # # Locate the standard TurtleBot3 Cartographer package
    # cartographer_pkg_dir = get_package_share_directory('turtlebot3_cartographer')
    # cartographer_launch_dir = os.path.join(cartographer_pkg_dir, 'launch')

    # # Include external launch
    # cartographer_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(cartographer_launch_dir, 'cartographer.launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items()
    # )

    lidar_distance_node = Node(
        package='turtlebot3_subgoal',          
        executable='lidar_distance',     
        name='lidar_distance',
        output='screen'
    )

    fuzzy_planner_node = Node(
        package='turtlebot3_subgoal',   
        executable='fuzzy_planner',
        name='fuzzy_planner',
        output='screen'
    )


    return LaunchDescription([
        use_sim_time_arg,
        lidar_distance_node,
        fuzzy_planner_node,
    ])