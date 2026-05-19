import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                             ExecuteProcess, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', robot_type_str, robot_type_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true',  
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )]


def generate_launch_description():

    load_gripper    = LaunchConfiguration('load_gripper')
    franka_hand     = LaunchConfiguration('franka_hand')
    robot_type      = LaunchConfiguration('robot_type')
    namespace       = LaunchConfiguration('namespace')

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'),
                             'rviz', 'visualize_franka.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        namespace=namespace,
        arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    # ── Controllers ───────────────────────────────────────────────────────────

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # <<< ADDED — spawns our impedance controller after joint_state_broadcaster
    impedance_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'joint_impedance_sim_controller',
            '--controller-manager-timeout', '30',
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_gazebo_bringup'),
            'config', 'franka_gazebo_controllers.yaml'
        ])],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('franka_hand',  default_value='franka_hand'),
        DeclareLaunchArgument('robot_type',   default_value='fr3'),
        DeclareLaunchArgument('namespace',    default_value=''),

        gazebo_empty_world,
        OpaqueFunction(function=get_robot_description,
                       args=[robot_type, load_gripper, franka_hand]),
        rviz,
        spawn,

        # Chain: spawn → joint_state_broadcaster → impedance_controller
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster],
        )),
        RegisterEventHandler(OnProcessExit(             
            target_action=joint_state_broadcaster,
            on_exit=[impedance_controller],
        )),

        # Keep joint_state_publisher for RViz visualization
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{'source_list': ['joint_states'], 'rate': 30}],
        ),
    ])