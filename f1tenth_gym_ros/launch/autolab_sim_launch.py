# MIT License
#
# Same terms as gym_bridge_launch.py (Copyright (c) 2020 Hongrui Zheng).
#
# 자동차공학실험 시연 트랙(autolab_track) 3D 시뮬레이션.
#   - gym_bridge: 2D 코어로 차량 동역학, 라이다(/scan), 충돌을 계산한다
#   - fake_camera: 같은 트랙의 3D 장면을 렌더링해 /flir_camera/image_raw 를 낸다
#   - track_markers: RViz 에 바닥·테이프·덕트를 3D 로 그린다
#
#   ros2 launch f1tenth_gym_ros autolab_sim_launch.py
#   ros2 launch f1tenth_gym_ros autolab_sim_launch.py sx:=-1.2 sy:=-3.5 stheta:=-1.5708 camera_rate:=15
#
# 라이다는 기본으로 YDLIDAR TEA 를 흉내 낸다 (시야 300°, 1250 빔, 후륜축에서 0.27 m).
# 빈 인자는 track.yaml(시작 자세)과 config/sim.yaml(나머지) 값을 쓴다.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    share = get_package_share_directory('f1tenth_gym_ros')
    config = os.path.join(share, 'config', 'sim.yaml')
    arg = lambda n: LaunchConfiguration(n).perform(context).strip()
    on = lambda n: arg(n).lower() not in ('false', '0', 'no')

    map_path = os.path.join(share, 'maps', 'autolab_track')
    track_file = map_path + '.track.yaml'
    with open(track_file) as f:
        start = yaml.safe_load(f)['start']

    overrides = {
        'map_path': map_path,
        'map_img_ext': '.png',
        'num_agent': 1,
        'sx': float(arg('sx') or start['x']),
        'sy': float(arg('sy') or start['y']),
        'stheta': float(arg('stheta') or start['theta']),
        'scan_fov': float(arg('scan_fov')),
        'scan_beams': int(arg('scan_beams')),
        'scan_distance_to_base_link': float(arg('scan_distance_to_base_link')),
    }

    actions = [
        Node(package='f1tenth_gym_ros', executable='gym_bridge', name='bridge',
             parameters=[config, overrides]),
        Node(package='nav2_map_server', executable='map_server',
             parameters=[{'yaml_filename': map_path + '.yaml'}, {'topic': 'map'},
                         {'frame_id': 'map'}, {'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': ['map_server']}]),
    ]
    if on('camera'):
        actions.append(Node(
            package='f1tenth_gym_ros', executable='fake_camera', name='fake_camera', output='screen',
            parameters=[os.path.join(share, 'config', 'autolab_camera.yaml'),
                        {'track_file': track_file, 'rate_hz': float(arg('camera_rate'))}]))
    if on('rviz'):
        actions.append(Node(package='f1tenth_gym_ros', executable='track_markers', name='track_markers',
                            parameters=[{'track_file': track_file}]))
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='ego_robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', os.path.join(share, 'launch', 'ego_racecar.xacro')])}],
            remappings=[('/robot_description', 'ego_robot_description')]))
        actions.append(ExecuteProcess(
            cmd=['bash', '-lc',
                 'exec rviz2 -d "$1" --ros-args -r __node:=rviz '
                 "2> >(grep -v -E 'indexed_8bit_image\\.(vert|frag)|active samplers with a different type refer to the same texture image unit' >&2)",
                 'rviz2', os.path.join(share, 'launch', 'autolab.rviz')],
            name='rviz', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sx', default_value='', description='시작 x [m]. 비우면 track.yaml'),
        DeclareLaunchArgument('sy', default_value='', description='시작 y [m]. 비우면 track.yaml'),
        DeclareLaunchArgument('stheta', default_value='', description='시작 yaw [rad]. 비우면 track.yaml'),
        DeclareLaunchArgument('scan_fov', default_value='5.236', description='라이다 시야 [rad]. TEA 300°'),
        DeclareLaunchArgument('scan_beams', default_value='1250', description='라이다 빔 수. TEA 20 Hz 약 0.24°'),
        DeclareLaunchArgument('scan_distance_to_base_link', default_value='0.27',
                              description='후륜축에서 라이다까지 [m] (팀 mount.yaml)'),
        DeclareLaunchArgument('camera', default_value='true', description='가상 카메라 실행'),
        DeclareLaunchArgument('camera_rate', default_value='20.0', description='가상 카메라 Hz'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 와 3D 마커 실행'),
        OpaqueFunction(function=_launch_setup),
    ])
