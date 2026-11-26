# autolab_track: 자동차공학실험 시연 트랙 3D 시뮬레이션

2026-11-26 트랙 사진 4장(600 mm 타일로 원근 보정)에서 복원한 트랙이에요. 기존 2D 엔진은 그대로 두고, 같은 트랙의 3D 장면을 가상 카메라와 RViz에 더했어요.

| 무엇 | 어디서 | 내용 |
| --- | --- | --- |
| 차량 동역학, 라이다, 충돌 | gym_bridge (기존 C++ 코어) | `maps/autolab_track.png` 2D 점유맵. 라이다 높이의 덕트, 장애물, 배경 물체 |
| 카메라 영상 | `fake_camera` 노드 | 바닥, 노란 테이프, 덕트 원통, 장애물을 레이캐스트로 렌더링해 `/flir_camera/image_raw` 발행 |
| 3D 화면 | `track_markers` 노드 | RViz `/track_markers` MarkerArray |

세 가지 모두 `f1tenth_gym_ros/maps/autolab_track.track.yaml` 한 파일에서 나와요.

## 실행

```bash
cd ~/f1sim_ms
colcon build --symlink-install --base-paths f1tenth_gym_ros   # 처음 한 번 (새 노드 등록)
source install/setup.bash
ros2 launch f1tenth_gym_ros autolab_sim_launch.py
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `sx`, `sy`, `stheta` | track.yaml 의 `start` | 시작 자세. 오른쪽 덕트 통로에서 위(+y)를 보고 반시계 |
| `scan_fov`, `scan_beams` | 5.236, 1250 | YDLIDAR TEA 300°, 20 Hz 분해능 |
| `scan_distance_to_base_link` | 0.27 | 후륜축에서 라이다까지 (팀 mount.yaml) |
| `camera`, `camera_rate` | true, 20.0 | 가상 카메라. 느리면 rate 를 낮추거나 `config/autolab_camera.yaml` 의 `solid_scale` 을 0.35 로 |
| `rviz` | true | RViz, 3D 마커 |

차선 구간에서 바로 시작하려면:

```bash
ros2 launch f1tenth_gym_ros autolab_sim_launch.py sx:=-1.2 sy:=-3.4 stheta:=-1.5708
```

팀 스택(2026AUTOLAB_team2)과 같이 돌릴 때는 두 터미널 모두 같은 `ROS_DOMAIN_ID` 를 쓰고, 팀 쪽은 `profile:=sim` 으로 띄워요. 토픽은 `/scan`, `/ego_racecar/odom`, `/drive`, `/flir_camera/image_raw` 로 맞춰져 있어요.

## 트랙 고치기

1. `f1tenth_gym_ros/maps/autolab_track.track.yaml` 의 좌표를 고쳐요 (map 프레임, m, 타일 줄눈이 0.6 m 배수).
2. 2D 점유맵을 다시 만들어요.
   ```bash
   python3 tools/autolab_track/build_track.py
   ```
3. 시뮬을 다시 띄워요. 카메라와 3D 마커는 track.yaml 을 직접 읽어서 따로 할 일이 없어요.

`tools/autolab_track/autolab_track_preview.png` 로 위에서 본 모습을 확인할 수 있어요.

## 정확도와 한계

- 덕트와 테이프 위치는 사진 원근 보정이라 ±10 cm 안팎이에요. 전시차 옆 오른쪽 바깥 덕트와 차량(scenery)은 가려진 부분이 있어 더 거칠어요. 줄자로 잰 값이 있으면 바꿔 주세요.
- 바닥, 테이프, 덕트 색은 폰 사진에서 잰 값이에요. FLIR 로 찍은 프레임에서 다시 재면 HSV 튜닝이 더 실제에 가까워져요.
- 덕트와 상자는 절반 해상도로 그려서 경계가 조금 계단져요. 차선 검출에 쓰는 바닥과 테이프는 전체 해상도예요.
- 그림자, 반사, 조명 변화, 모션 블러는 없어요. 노란 A형 안내판처럼 트랙 밖의 노란 물체도 넣지 않았어요.
