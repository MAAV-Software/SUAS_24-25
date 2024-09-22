import os
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


with open(os.path.join(get_package_share_directory('simulation'),'config', 'config.yaml'), "r") as config_stream:
    config = yaml.safe_load(config_stream)

def generate_launch_description():

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(get_package_share_directory('simulation'), 'world', 'empty.world'),'gui': LaunchConfiguration("gui")}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('simulation'),'rviz', 'rviz_config.rviz')],
    )

    spawn_bike = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        output='screen',
        arguments=("-x {x} -y {y} -z {z} -R {roll} -P {pitch} -Y {yaw} -entity bike -topic robot_description".format(
            x=config["initial_position"]["x"],
            y=config["initial_position"]["y"],
            z=config["initial_position"]["z"],
            roll=config["initial_position"]["roll"],
            pitch=config["initial_position"]["pitch"],
            yaw=config["initial_position"]["yaw"])).split())

    if config["training_wheels"] == True:
        bike_urdf_file = "bike_training_wheels.urdf"
    else:
        bike_urdf_file = "bike_description.urdf"

    bike_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}, {"robot_description": xacro.process(os.path.join(get_package_share_directory('simulation'), "urdf", bike_urdf_file))}])

    bike_simulation_node = Node(
        package = "simulation",
        executable = "bike_simulation_node",
        name="bike_simulation_node")
    
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        simulation,
        rviz,
        spawn_bike,
        bike_state_publisher,
        bike_simulation_node
    ])