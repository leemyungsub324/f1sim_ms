# 실행 방법

이 저장소의 시뮬레이터를 설치하고 실행하는 방법이에요. Ubuntu 22.04와 ROS 2 Humble을 기준으로 해요.

## 1. 설치 (처음 한 번)

```bash
cd ~/f1sim_ms
./install.sh
```

`install.sh`가 하는 일은 두 가지예요.

1. `pip3 install -e .`로 `f110_gym`을 설치하고 C++ 시뮬레이션 코어를 컴파일해요.
2. ROS 2가 있으면 `rosdep`으로 의존성을 설치하고 `colcon build`로 `f1tenth_gym_ros` 브리지를 빌드해요.

`rosdep`을 한 번도 안 썼다면 먼저 다음을 실행해요.

```bash
sudo rosdep init && rosdep update
```

### 코드를 고친 뒤 다시 빌드

새 노드를 추가했거나 `setup.py`를 바꿨으면 다시 빌드해요. `--symlink-install`로 빌드했다면 Python 코드, launch 파일, yaml만 바꿨을 때는 다시 빌드하지 않아도 돼요.

```bash
cd ~/f1sim_ms
colcon build --symlink-install --base-paths f1tenth_gym_ros
```

## 2. 터미널 준비

새 터미널을 열 때마다 아래 두 줄을 실행해요.

```bash
source /opt/ros/humble/setup.bash
source ~/f1sim_ms/install/setup.bash
```

## 3. autolab 시연 트랙 실행 (3D 카메라 포함)

자동차공학실험 시연 트랙이에요. 2D 시뮬레이터(차량 동역학, 라이다, 충돌)에 가상 카메라와 RViz 3D 화면을 더했어요.

```bash
ros2 launch f1tenth_gym_ros autolab_sim_launch.py
```

실행하면 다음이 같이 떠요.

| 노드 | 하는 일 |
| --- | --- |
| `gym_bridge` | 차량 동역학, 라이다 `/scan`, 충돌 계산 |
| `map_server` | 2D 점유맵 `/map` |
| `fake_camera` | 가상 카메라 영상 `/flir_camera/image_raw` |
| `track_markers` | RViz 3D 트랙 `/track_markers` |
| `rviz2` | 화면 |

### 자주 쓰는 인자

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `sx`, `sy`, `stheta` | `autolab_track.track.yaml`의 `start` | 시작 위치 [m]와 방향 [rad] |
| `scan_fov` | `5.236` | 라이다 시야 [rad] (YDLIDAR TEA 300°) |
| `scan_beams` | `1250` | 라이다 빔 수 |
| `scan_distance_to_base_link` | `0.27` | 후륜축에서 라이다까지 거리 [m] |
| `camera` | `true` | 가상 카메라를 켤지 |
| `camera_rate` | `20.0` | 가상 카메라 주기 [Hz] |
| `rviz` | `true` | RViz와 3D 마커를 켤지 |

예시:

```bash
# 차선 구간에서 바로 시작
ros2 launch f1tenth_gym_ros autolab_sim_launch.py sx:=-1.2 sy:=-3.4 stheta:=-1.5708

# 컴퓨터가 느릴 때: 카메라 주기를 낮춰요
ros2 launch f1tenth_gym_ros autolab_sim_launch.py camera_rate:=10.0

# 카메라와 RViz 없이 2D만
ros2 launch f1tenth_gym_ros autolab_sim_launch.py camera:=false rviz:=false
```

카메라가 여전히 느리면 `f1tenth_gym_ros/config/autolab_camera.yaml`에서 `solid_scale`을 `0.35`로 낮춰요.

### 카메라 영상 확인

```bash
ros2 run rqt_image_view rqt_image_view /flir_camera/image_raw
```

## 4. 기본 트랙 실행 (levine)

카메라 없는 원래 2D 시뮬레이터예요.

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

맵, 차량 수, 라이다 같은 설정은 `f1tenth_gym_ros/config/sim.yaml`에서 바꿔요.

## 5. 차 움직이기

### 키보드로

다른 터미널에서 실행해요. `sim.yaml`의 `kb_teleop: True`일 때 동작해요.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 직접 명령을 보내서

```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.0}}" -r 20
```

### 위치 다시 잡기

RViz 위쪽의 **2D Pose Estimate**를 누르고 맵 위를 클릭한 채 끌어서 방향을 정해요.

## 6. 주요 토픽

| 토픽 | 타입 | 방향 | 내용 |
| --- | --- | --- | --- |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | 구독 | 조향각과 속도 명령 |
| `/scan` | `sensor_msgs/LaserScan` | 발행 | 2D 라이다 |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | 발행 | 차 위치와 속도 |
| `/flir_camera/image_raw` | `sensor_msgs/Image` | 발행 | 가상 카메라 영상 (autolab만) |
| `/flir_camera/camera_info` | `sensor_msgs/CameraInfo` | 발행 | 카메라 내부 파라미터 (autolab만) |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 구독 | 위치 재설정 |
| `/cmd_vel` | `geometry_msgs/Twist` | 구독 | 키보드 조종 |

## 7. 팀 스택(2026AUTOLAB_team2)과 같이 돌리기

1. 시뮬레이터 터미널과 팀 스택 터미널 모두 같은 `ROS_DOMAIN_ID`를 써요.
   ```bash
   export ROS_DOMAIN_ID=0
   ```
2. 시뮬레이터를 먼저 띄워요.
   ```bash
   ros2 launch f1tenth_gym_ros autolab_sim_launch.py
   ```
3. 팀 스택은 `profile:=sim`으로 띄워요. 토픽 이름(`/scan`, `/ego_racecar/odom`, `/drive`, `/flir_camera/image_raw`)이 이미 맞춰져 있어요.

## 8. 트랙 고치기

1. `f1tenth_gym_ros/maps/autolab_track.track.yaml`의 좌표를 고쳐요. map 프레임 기준이고 단위는 m예요.
2. 2D 점유맵을 다시 만들어요.
   ```bash
   python3 tools/autolab_track/build_track.py
   ```
3. 시뮬레이터를 다시 띄워요. 카메라와 3D 마커는 track.yaml을 직접 읽기 때문에 따로 할 일이 없어요.

위에서 본 모습은 `tools/autolab_track/autolab_track_preview.png`로 확인해요. 자세한 내용은 [tools/autolab_track/README.md](tools/autolab_track/README.md)에 있어요.

## 문제가 생기면

| 증상 | 해결 |
| --- | --- |
| `Package 'f1tenth_gym_ros' not found` | `source ~/f1sim_ms/install/setup.bash`를 했는지 확인해요 |
| `executable 'fake_camera' not found` | 새 노드가 등록되지 않은 상태예요. `colcon build`를 다시 해요 |
| 카메라 영상이 끊기거나 느려요 | `camera_rate`나 `solid_scale`을 낮춰요 |
| 팀 스택이 토픽을 못 받아요 | 두 터미널의 `ROS_DOMAIN_ID`가 같은지 확인해요 |
