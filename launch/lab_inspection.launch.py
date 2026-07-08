import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('lab_inspection')
    
    # 1. 声明机器人初始坐标参数
    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.0', description='初始X坐标')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.0', description='初始Y坐标')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='0.0', description='初始朝向弧度')

    # 2. 联动拉起底盘、相机、激光雷达驱动组件(OriginBot 原生底层包)
    originbot_bringup_dir = get_package_share_directory('originbot_bringup')
    robot_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(originbot_bringup_dir, 'launch', 'originbot.launch.py')
        ),
        launch_arguments={
            'use_camera': 'true',
            'use_lidar': 'true',
            'use_imu': 'true'
        }.items()
    )

    # 2.5 补发 base_link→laser_link 静态TF（VP100驱动内使用了ROS1格式参数，在ROS2中无效）
    #     其他 TF（map→odom、odom→base_footprint）由 AMCL 和底盘驱动各自负责
    #     里程计兜底由 navigation_node 内置发布，无需额外节点
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser_fix',
        arguments=['--x', '0', '--y', '0', '--z', '0.02',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_link'],
    )

    # 3. 载入先前已校准完毕的导航全局参数
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    yolov8_params_file = os.path.join(pkg_dir, 'config', 'yolov8_config.yaml')
    map_file_path = os.path.join(pkg_dir, 'maps', 'lab_map.yaml')

    # 诊断: 打印关键文件路径，方便排查路径问题
    map_file_log = LogInfo(msg=['地图文件路径: ', map_file_path])
    nav2_params_log = LogInfo(msg=['导航参数文件路径: ', nav2_params_file])

    # 4. 启动 Nav2 导航核心节点集
    # 注意: map_server 也需要 nav2_params.yaml 以获取 use_sim_time 等必要参数
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_file_path}]
    )
    amcl = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[nav2_params_file])
    planner_server = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[nav2_params_file])
    controller_server = Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[nav2_params_file])
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[nav2_params_file])
    behavior_server = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[nav2_params_file])

    # 5. 使用 Nav2 官方生命周期管理器（替代脆弱的 bash 脚本）
    #    按顺序激活: map_server → amcl → 其他节点
    #    bond_timeout 从 10.0s 降至 5.0s，正常节点激活只需 <1s
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file, {
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'planner_server',
                           'controller_server', 'behavior_server', 'bt_navigator'],
            'bond_timeout': 15.0,               # ★ 从5.0提高到15.0，map_server加载地图需要时间
            'attempt_respawn_reconnection': True,
        }]
    )

    # 6. 视觉识别模块 (双模型ONNX并集 + 华为云上报)
    vision_core_node = Node(
        package='lab_inspection',
        executable='vision_py.py',
        name='vision_node',
        output='screen',
        parameters=[yolov8_params_file]
    )

    # 7. 拉起你队友做好的工位导航节点
    navigation_coordinator_node = Node(
        package='lab_inspection',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'frame_id': 'map'
        }]
    )

    return LaunchDescription([
        # 禁用 loaned messages 消除 originbot_base 警告
        SetEnvironmentVariable('ROS_DISABLE_LOANED_MESSAGES', '1'),

        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,

        # 诊断信息
        map_file_log,
        nav2_params_log,
        
        # 硬件基础（立即启动）
        robot_hardware_launch,
        laser_tf_node,                   # 修复 VP100 驱动 ROS1 格式 TF 在 ROS2 无效的问题
        
        # 导航状态机（延迟 20s，等 VP100 激光雷达初始化完毕）
        TimerAction(period=20.0, actions=[
            map_server,
            amcl,
            planner_server,
            controller_server,
            bt_navigator,
            behavior_server,
            lifecycle_manager,
        ]),
        
        # 核心业务
        vision_core_node,
        navigation_coordinator_node
    ])