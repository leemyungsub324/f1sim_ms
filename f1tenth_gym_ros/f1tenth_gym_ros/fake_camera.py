"""가상 카메라: 시뮬레이터의 차 자세로 autolab_track 3D 장면을 렌더링해 카메라 영상을 낸다.

/ego_racecar/odom → /flir_camera/image_raw (bgr8), /flir_camera/camera_info

실제 차의 FLIR BFS-U3-23S3C 와 같은 내부 파라미터(ost.yaml), 렌즈 왜곡, 장착 위치를 쓴다.
그래서 팀 레포의 lane_detector 가 실제 영상처럼 왜곡 보정 → BEV → 차선 검출을 할 수 있다.
렌더링은 track_scene.Renderer 에 있다 (바닥 평면 + 덕트 원통 + 상자 레이캐스트).
"""
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from f1tenth_gym_ros.track_scene import Renderer, camera_from_ost, load_track

# 팀 레포 bringup/config/ost.yaml (풀해상도 1920x1200) 값
OST_K = [1044.310885, 0.0, 967.285496, 0.0, 1040.725825, 592.034680, 0.0, 0.0, 1.0]
OST_D = [-0.326346, 0.114363, -0.000872, -0.000147, -0.019324]


class FakeCamera(Node):

    def __init__(self):
        super().__init__('fake_camera')
        default_track = os.path.join(get_package_share_directory('f1tenth_gym_ros'),
                                     'maps', 'autolab_track.track.yaml')
        p = {
            'track_file': default_track,
            'odom_topic': '/ego_racecar/odom',
            'image_topic': '/flir_camera/image_raw',
            'info_topic': '/flir_camera/camera_info',
            'frame_id': 'camera',
            'rate_hz': 20.0,
            'width': 480, 'height': 300,             # 비닝 4 (실제 차와 같다)
            'calib_width': 1920, 'calib_height': 1200,
            'K': OST_K, 'D': OST_D,
            'cam_x': 0.236, 'cam_y': 0.0, 'cam_z': 0.177,  # 후륜축 기준 (팀 safe.yaml lane_detector)
            'pitch_deg': 6.0,                         # 아래가 +
            'solid_scale': 0.5,                       # 덕트·상자를 그리는 해상도 배율. 낮추면 빠르다
            'noise': 2.0,                             # 화소 잡음 표준편차
            'max_range': 12.0,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value

        track = load_track(g('track_file'))
        cam = camera_from_ost(g('K'), g('D'), (g('calib_width'), g('calib_height')),
                              (g('width'), g('height')), g('cam_x'), g('cam_y'), g('cam_z'),
                              g('pitch_deg'))
        self.renderer = Renderer(track, cam, solid_scale=g('solid_scale'), max_range=g('max_range'))
        self.noise = g('noise')
        self.frame_id = g('frame_id')
        self.info = self._camera_info(cam)
        self.pose = None
        self.stamp = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.img_pub = self.create_publisher(Image, g('image_topic'), qos)
        self.info_pub = self.create_publisher(CameraInfo, g('info_topic'), 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        self.create_timer(1.0 / g('rate_hz'), self.on_timer)
        self.slow_since = None
        self.get_logger().info(f"트랙 {g('track_file')}, {cam.width}x{cam.height} @ {g('rate_hz')} Hz")

    def _camera_info(self, cam):
        info = CameraInfo()
        info.width, info.height = cam.width, cam.height
        info.distortion_model = 'plumb_bob'
        info.d = [float(v) for v in cam.D]
        info.k = [float(v) for v in cam.K.ravel()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = np.zeros((3, 4))
        P[:, :3] = cam.K
        info.p = [float(v) for v in P.ravel()]
        return info

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.stamp = msg.header.stamp

    def on_timer(self):
        if self.pose is None:
            return
        t0 = time.monotonic()
        img = self.renderer.render(*self.pose, noise=self.noise)
        dt = time.monotonic() - t0
        msg = Image()
        msg.header.stamp = self.stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.img_pub.publish(msg)
        self.info.header = msg.header
        self.info_pub.publish(self.info)
        period = 1.0 / self.get_parameter('rate_hz').value
        if dt > period:
            self.get_logger().warning(
                f'렌더링 {dt * 1000:.0f} ms 가 주기 {period * 1000:.0f} ms 보다 길다. '
                'rate_hz 나 solid_scale 을 낮춘다', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
