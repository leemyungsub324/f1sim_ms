"""autolab_track 3D 장면을 RViz MarkerArray 로 보여 준다. (/track_markers, map 프레임)

바닥(밝은/어두운 타일), 노란 테이프, 덕트 원통, 장애물, 배경 물체, 벽을 그린다.
렌더링과 같은 track.yaml 을 읽으므로 카메라 영상과 RViz 3D 화면이 일치한다.
"""
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_gym_ros.track_scene import _boxes, load_track, resample


def rgba(bgr, a=1.0):
    return ColorRGBA(r=bgr[2] / 255.0, g=bgr[1] / 255.0, b=bgr[0] / 255.0, a=a)


def pts(arr):
    return [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in arr]


def floor_marker(track, cell=0.15):
    f = track['floor']
    (x0, x1), (y0, y1) = track['bounds']['x'], track['bounds']['y']
    b = f['boundary']
    p1, p2, ds = (np.array(b[k], float) for k in ('p1', 'p2', 'dark_side'))
    d = p2 - p1
    side = lambda x, y: d[0] * (y - p1[1]) - d[1] * (x - p1[0])
    dark_sign = np.sign(side(*ds))
    m = Marker(type=Marker.TRIANGLE_LIST, ns='floor', id=0)
    m.scale.x = m.scale.y = m.scale.z = 1.0
    light, dark = rgba(f['light_bgr']), rgba(f['dark_bgr'])
    verts, cols = [], []
    for x in np.arange(x0, x1, cell):
        for y in np.arange(y0, y1, cell):
            c = dark if np.sign(side(x + cell / 2, y + cell / 2)) == dark_sign else light
            a, bq, cq, dq = (x, y, 0), (x + cell, y, 0), (x + cell, y + cell, 0), (x, y + cell, 0)
            verts += [a, bq, cq, a, cq, dq]
            cols += [c] * 6
    m.points = pts(verts)
    m.colors = cols
    return m


def tape_marker(track):
    t = track['tapes']
    m = Marker(type=Marker.TRIANGLE_LIST, ns='tapes', id=0)
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = rgba(t['bgr'])
    w, z, verts = t['width'] / 2, 0.004, []
    for line in t['lines'].values():
        p = resample(line, 0.05)
        tg = np.gradient(p, axis=0)
        tg /= np.linalg.norm(tg, axis=1, keepdims=True)
        n = np.stack([-tg[:, 1], tg[:, 0]], 1) * w
        L, R = p + n, p - n
        for i in range(len(p) - 1):
            a, b, c, d = (*L[i], z), (*L[i + 1], z), (*R[i + 1], z), (*R[i], z)
            verts += [a, b, c, a, c, d]
    m.points = pts(verts)
    return m


def duct_marker(track, sides=12):
    du = track['ducts']
    r = du['radius']
    m = Marker(type=Marker.TRIANGLE_LIST, ns='ducts', id=0)
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = rgba(du['bgr'])
    ang = np.linspace(0, 2 * np.pi, sides + 1)
    verts = []
    for line in du['lines'].values():
        p = resample(line, 0.05)
        tg = np.gradient(p, axis=0)
        tg /= np.linalg.norm(tg, axis=1, keepdims=True)
        n = np.stack([-tg[:, 1], tg[:, 0]], 1)
        rings = np.stack([np.stack([p[:, 0] + r * np.cos(a) * n[:, 0], p[:, 1] + r * np.cos(a) * n[:, 1],
                                    np.full(len(p), r + r * np.sin(a))], 1) for a in ang], 1)
        for i in range(len(p) - 1):
            for k in range(sides):
                a, b = rings[i, k], rings[i, k + 1]
                c, d = rings[i + 1, k + 1], rings[i + 1, k]
                verts += [a, b, c, a, c, d]
    m.points = pts(verts)
    return m


def box_markers(track):
    out = []
    names = len(track.get('obstacles', []))
    for i, (c, half, yaw, bgr) in enumerate(_boxes(track)):
        m = Marker(type=Marker.CUBE, ns='obstacles' if i < names else 'scenery', id=i)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, c)
        m.pose.orientation.z, m.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        m.scale.x, m.scale.y, m.scale.z = (float(2 * h) for h in half)
        m.color = rgba(bgr, 1.0 if i < names else 0.35)
        out.append(m)
    return out


class TrackMarkers(Node):

    def __init__(self):
        super().__init__('track_markers')
        default_track = os.path.join(get_package_share_directory('f1tenth_gym_ros'),
                                     'maps', 'autolab_track.track.yaml')
        self.declare_parameter('track_file', default_track)
        self.declare_parameter('frame_id', 'map')
        track = load_track(self.get_parameter('track_file').value)
        frame = self.get_parameter('frame_id').value
        self.msg = MarkerArray()
        for m in [floor_marker(track), tape_marker(track), duct_marker(track)] + box_markers(track):
            m.header.frame_id = frame
            m.action = Marker.ADD
            if m.pose.orientation.w == 0.0:
                m.pose.orientation.w = 1.0
            self.msg.markers.append(m)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/track_markers', qos)
        self.pub.publish(self.msg)
        self.create_timer(5.0, lambda: self.pub.publish(self.msg))
        self.get_logger().info(f'마커 {len(self.msg.markers)} 개 발행 (/track_markers)')


def main(args=None):
    rclpy.init(args=args)
    node = TrackMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
