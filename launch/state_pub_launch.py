from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
import yaml

def load_config():
    """加载配置文件，返回路径配置字典"""
    config_file = os.getenv('CONFIG_FILE_PATH')
    if os.path.exists(config_file):
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
            return config.get('paths', {})
    else:
        print(f"配置文件 {config_file} 不存在，使用默认路径")
        return {}

def generate_launch_description():
    # 加载路径配置
    paths = load_config()

    # 获取 URDF 文件路径，若无配置则使用默认绝对路径
    urdf_path = paths.get('urdf_file', '<if urdf_file is not found, type your absolute path here!>')
    
    # 打开并读取 URDF 文件内容
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    return LaunchDescription([
        # 启动 joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
        ),

        # 启动 robot_state_publisher，将其订阅的 joint_states 改为 /mujoco_joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
            remappings=[
                ('/joint_states', '/mujoco_joint_states')  
            ]
        ),

        # 启动 mj2rviz 的 pub_test 节点
        Node(
            package='mj2rviz',
            executable='pub_test',
        ),

        # 启动 mj2rviz 的 simulate 节点
        Node(
            package='mj2rviz',
            executable='simulate',
        ),

        ExecuteProcess(
            cmd=[
                'rviz2',
                '--ros-args',
                '-r', '/joint_states:=/mujoco_joint_states',
                '-r', '/tf:=/mujoco_tf',
            ],
            output='screen'
        ),

        # 添加 static_transform_publisher，将 world 和 pelvis 连接
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0', '0', '0', '0',  # 变换参数
                'world', 'pelvis'  # 父子坐标系
            ],
            output='screen'
        )
    ])
