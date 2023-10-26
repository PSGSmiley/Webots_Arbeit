import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from webots_ros2_driver.utils import controller_url_prefix
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    package_dir = get_package_share_directory('tracked_robot')
    robot_description_path = os.path.join(package_dir, 'resource', 'tracked.urdf')
    default_model_path = os.path.join(package_dir, 'description/tracked_robot_description.urdf')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=True)
    use_cartographer = LaunchConfiguration('cartographer', default=False)
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'hall.wbt'),
        ros2_supervisor=True
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    lidar_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.37', '3.141592653589793', '0', '0', 'base_link', 'base_laser'],
    )

    my_robot_driver = WebotsController(
        robot_name='tracked',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    tool_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_tools_launch.py')
        ),
        launch_arguments={
            'slam': use_slam,
            'nav': use_nav,
            'rviz': use_rviz,
            'cartographer': use_cartographer,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        webots,
        webots._supervisor,
        my_robot_driver,
        lidar_publisher,
        tool_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
