"""autolab_track 3D 장면: track.yaml 을 읽어 바닥 텍스처, 2D 점유맵, 레이캐스트 렌더러를 만든다.

ROS 없이 numpy, OpenCV, PyYAML 만으로 돈다. 쓰는 곳:
- tools/autolab_track/build_track.py: 2D 점유맵 png/yaml 생성
- fake_camera 노드: 차 자세에서 카메라 영상 렌더링
- track_markers 노드: RViz 3D 마커

좌표는 map 프레임(m, x 오른쪽, y 위, z 위)이다.
"""
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


def load_track(path):
    with open(path) as f:
        return yaml.safe_load(f)


def resample(points, step):
    """폴리라인을 step 간격으로 다시 찍는다. 끝점은 그대로 둔다."""
    p = np.asarray(points, float)
    seg = np.diff(p, axis=0)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    t = np.append(np.arange(0.0, s[-1], step), s[-1])
    return np.stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])], 1)


def _boxes(track):
    """장애물, 배경 물체, 맵 가장자리 벽을 (center xyz, half size, yaw, bgr) 목록으로."""
    out = []
    for b in track.get('obstacles', []) + track.get('scenery', []):
        sx, sy, sz = b['size']
        out.append(((b['center'][0], b['center'][1], sz / 2), (sx / 2, sy / 2, sz / 2),
                    b.get('yaw', 0.0), b['bgr']))
    (x0, x1), (y0, y1) = track['bounds']['x'], track['bounds']['y']
    h, t = track.get('wall_height', 2.5), 0.1
    wall = [70, 70, 75]
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    for c, half in (((cx, y0, h / 2), (hx, t, h / 2)), ((cx, y1, h / 2), (hx, t, h / 2)),
                    ((x0, cy, h / 2), (t, hy, h / 2)), ((x1, cy, h / 2), (t, hy, h / 2))):
        out.append((c, half, 0.0, wall))
    return out


class Grid:
    """map 좌표 ↔ 이미지 픽셀. 이미지 위쪽이 +y (ROS 맵 png 규약)."""

    def __init__(self, track, res):
        (self.x0, x1), (y0, self.y1) = track['bounds']['x'], track['bounds']['y']
        self.res = res
        self.w = int(round((x1 - self.x0) / res))
        self.h = int(round((self.y1 - y0) / res))
        self.origin = [self.x0, y0, 0.0]

    def to_px(self, xy):
        xy = np.asarray(xy, float)
        return np.stack([(xy[..., 0] - self.x0) / self.res, (self.y1 - xy[..., 1]) / self.res], -1)

    def centers(self):
        u, v = np.meshgrid(np.arange(self.w) + 0.5, np.arange(self.h) + 0.5)
        return self.x0 + u * self.res, self.y1 - v * self.res


def _draw_lines(img, grid, lines, width_m, value):
    for pts in lines:
        px = (grid.to_px(resample(pts, 0.02)) * 16).astype(np.int32)
        thick = max(1, int(round(width_m / grid.res)))
        cv2.polylines(img, [px], False, value, thick, cv2.LINE_AA, shift=4)


def _draw_boxes(img, grid, boxes, value):
    for c, half, yaw, _ in boxes:
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half[:2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        px = grid.to_px(corners @ rot.T + c[:2])
        cv2.fillPoly(img, [np.round(px).astype(np.int32)], value)


def occupancy_image(track, res=0.025, scan_height=0.11):
    """라이다 평면(scan_height)에서 본 2D 점유맵. 0 = 점유, 254 = 빈 곳. 테이프는 넣지 않는다."""
    grid = Grid(track, res)
    img = np.full((grid.h, grid.w), 254, np.uint8)
    r = track['ducts']['radius']
    # 누운 원통을 높이 scan_height 에서 자른 폭
    half = np.sqrt(max(r ** 2 - (r - min(scan_height, 2 * r)) ** 2, 0.0)) if scan_height < 2 * r else 0.0
    _draw_lines(img, grid, track['ducts']['lines'].values(), 2 * max(half, 0.5 * r), 0)
    _draw_boxes(img, grid, _boxes(track), 0)
    return img, grid


def floor_texture(track, res=0.01):
    """위에서 본 바닥 색(BGR). 타일 줄눈, 밝은/어두운 바닥, 노란 테이프, 덕트 접지 그림자."""
    grid = Grid(track, res)
    f = track['floor']
    X, Y = grid.centers()
    b = f['boundary']
    p1, p2, ds = (np.array(b[k], float) for k in ('p1', 'p2', 'dark_side'))
    d = p2 - p1
    side = lambda x, y: d[0] * (y - p1[1]) - d[1] * (x - p1[0])
    dark = np.sign(side(X, Y)) == np.sign(side(*ds))
    img = np.where(dark[..., None], np.array(f['dark_bgr'], np.float32),
                   np.array(f['light_bgr'], np.float32))
    rng = np.random.default_rng(0)
    img *= 1.0 + 0.03 * rng.standard_normal(img.shape[:2])[..., None].astype(np.float32)
    tile = f['tile_m']
    gu = (X % tile) < res  # 줄눈 1 px
    gv = (Y % tile) < res
    img[gu | gv] *= f['grout_darken']
    # 덕트 아래 접지 그림자 (지름보다 조금 넓게, 옅게)
    shade = np.zeros(img.shape[:2], np.uint8)
    _draw_lines(shade, grid, track['ducts']['lines'].values(), 2 * track['ducts']['radius'] + 0.08, 255)
    shade = cv2.GaussianBlur(shade, (0, 0), 0.03 / res).astype(np.float32) / 255.0
    img *= (1.0 - 0.35 * shade)[..., None]
    tape = np.zeros(img.shape[:2], np.uint8)
    _draw_lines(tape, grid, track['tapes']['lines'].values(), track['tapes']['width'], 255)
    a = (tape.astype(np.float32) / 255.0)[..., None]
    img = img * (1 - a) + np.array(track['tapes']['bgr'], np.float32) * a
    return np.clip(img, 0, 255).astype(np.uint8), grid


@dataclass
class Camera:
    """핀홀 + plumb_bob 왜곡. base_link(후륜축) 기준 장착 위치, pitch 는 아래가 +."""

    width: int
    height: int
    K: np.ndarray
    D: np.ndarray
    x: float
    y: float
    z: float
    pitch_deg: float


class Renderer:
    """바닥은 평면 교차 + 텍스처 샘플링, 덕트는 캡슐, 상자는 OBB 레이캐스트."""

    def __init__(self, track, cam, floor_res=0.01, solid_scale=0.5, max_range=12.0):
        self.track = track
        self.cam = cam
        self.max_range = max_range
        self.tex, self.tgrid = floor_texture(track, floor_res)
        self.bg = np.array([60, 60, 64], np.float32)
        self.d_base_full = self._rays(cam.width, cam.height)
        self.sw = max(1, int(cam.width * solid_scale))
        self.sh = max(1, int(cam.height * solid_scale))
        self.d_base_small = self._rays(self.sw, self.sh)
        # 작은 광선들을 방위각 순으로 정렬해 두면 물체마다 보이는 광선 구간만 잘라 계산할 수 있다
        az = np.arctan2(self.d_base_small[:, 1], self.d_base_small[:, 0])
        self.order = np.argsort(az)
        self.az_sorted = az[self.order]
        self.d_small_sorted = self.d_base_small[self.order]
        self._build_solids()
        self.rng = np.random.default_rng()

    def _rays(self, w, h):
        """픽셀마다 base_link 축에서의 광선 방향 (h*w, 3). 왜곡을 되돌려 실제 렌즈와 맞춘다."""
        c = self.cam
        sx, sy = w / c.width, h / c.height
        u, v = np.meshgrid((np.arange(w) + 0.5) / sx, (np.arange(h) + 0.5) / sy)
        pts = np.stack([u.ravel(), v.ravel()], 1).astype(np.float64)[:, None, :]
        n = cv2.undistortPoints(pts, c.K, c.D)[:, 0, :]
        d_opt = np.concatenate([n, np.ones((len(n), 1))], 1)
        d_body = np.stack([d_opt[:, 2], -d_opt[:, 0], -d_opt[:, 1]], 1)  # 광학축 → x 전방, y 왼쪽, z 위
        p = np.radians(c.pitch_deg)
        rot = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        d = d_body @ rot.T
        return (d / np.linalg.norm(d, axis=1, keepdims=True)).astype(np.float32)

    def _build_solids(self):
        du = self.track['ducts']
        r = du['radius']
        A, B, S = [], [], []
        for pts in du['lines'].values():
            p = resample(pts, 0.3)
            s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
            for i in range(len(p) - 1):
                A.append([p[i, 0], p[i, 1], r])
                B.append([p[i + 1, 0], p[i + 1, 1], r])
                S.append(s[i])
        self.capA = np.array(A, np.float32)
        self.capB = np.array(B, np.float32)
        self.capS = np.array(S, np.float32)
        self.cap_r = np.float32(r)
        self.duct_bgr = np.array(du['bgr'], np.float32)
        self.rib = du.get('rib_pitch_m', 0.06)
        self.boxes = _boxes(self.track)

    def render(self, x, y, yaw, noise=2.0):
        c = self.cam
        cy, sy = np.cos(yaw), np.sin(yaw)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], np.float32)
        origin = np.array([x + cy * c.x - sy * c.y, y + sy * c.x + cy * c.y, c.z], np.float32)

        # 바닥
        D = self.d_base_full @ rz.T
        with np.errstate(divide='ignore', invalid='ignore'):
            tg = np.where(D[:, 2] < -1e-4, -origin[2] / D[:, 2], np.inf)
        tg[tg > self.max_range] = np.inf
        gx = origin[0] + D[:, 0] * np.where(np.isfinite(tg), tg, 0)
        gy = origin[1] + D[:, 1] * np.where(np.isfinite(tg), tg, 0)
        px = self.tgrid.to_px(np.stack([gx, gy], 1)).astype(np.float32)
        mapx = px[:, 0].reshape(c.height, c.width) - 0.5
        mapy = px[:, 1].reshape(c.height, c.width) - 0.5
        img = cv2.remap(self.tex, mapx, mapy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        img = img.astype(np.float32)
        img[~np.isfinite(tg).reshape(c.height, c.width)] = self.bg

        # 덕트·상자 (낮은 해상도로 그리고 키운다)
        Ds = self.d_small_sorted @ rz.T
        t_sorted = np.full(len(Ds), np.inf, np.float32)
        col_sorted = np.zeros((len(Ds), 3), np.float32)
        self._yaw, self._origin = yaw, origin
        self._render_ducts(origin, Ds, t_sorted, col_sorted)
        self._render_boxes(origin, Ds, t_sorted, col_sorted)
        t_best = np.empty_like(t_sorted)
        col = np.empty_like(col_sorted)
        t_best[self.order] = t_sorted
        col[self.order] = col_sorted
        t_small = t_best.reshape(self.sh, self.sw)
        col_small = col.reshape(self.sh, self.sw, 3)
        t_up = cv2.resize(t_small, (c.width, c.height), interpolation=cv2.INTER_NEAREST)
        col_up = cv2.resize(col_small, (c.width, c.height), interpolation=cv2.INTER_NEAREST)
        front = t_up < tg.reshape(c.height, c.width)
        img[front] = col_up[front]

        if noise > 0:
            img += self.rng.normal(0, noise, img.shape).astype(np.float32)
        return np.clip(img, 0, 255).astype(np.uint8)

    def _ray_slice(self, pts_xy, margin):
        """물체 꼭짓점들(map xy)이 카메라에서 보이는 방위각 범위에 드는 광선 구간 (정렬 순서)."""
        rel = np.asarray(pts_xy, float) - self._origin[:2]
        dist = np.hypot(rel[:, 0], rel[:, 1])
        if dist.min() < margin * 1.5:
            return slice(0, len(self.az_sorted))  # 너무 가까우면 전부 본다
        az = np.arctan2(rel[:, 1], rel[:, 0]) - self._yaw
        az = (az + np.pi) % (2 * np.pi) - np.pi
        pad = np.arcsin(np.clip(margin / dist, 0, 1)).max()
        lo, hi = az.min() - pad, az.max() + pad
        if hi - lo > np.pi:  # 차 뒤를 감싸는 경우
            return slice(0, len(self.az_sorted))
        return slice(np.searchsorted(self.az_sorted, lo), np.searchsorted(self.az_sorted, hi))

    def _render_ducts(self, O, D_all, t_all, col_all):
        r = self.cap_r
        mid = (self.capA + self.capB) / 2
        rel = mid[:, :2] - O[:2]
        dist = np.hypot(rel[:, 0], rel[:, 1])
        fwd = np.array([D_all[:, 0].mean(), D_all[:, 1].mean()])
        fwd /= np.linalg.norm(fwd) + 1e-9
        keep = (dist < self.max_range) & ((rel @ fwd) > -0.6)
        light = np.array([0.3, 0.2, 0.93], np.float32)
        light /= np.linalg.norm(light)
        for A, B, s0 in zip(self.capA[keep], self.capB[keep], self.capS[keep]):
            sl = self._ray_slice([A[:2], B[:2]], r)
            if sl.stop <= sl.start:
                continue
            D, t_best, col = D_all[sl], t_all[sl], col_all[sl]
            ba = B - A
            oa = O - A
            baba = ba @ ba
            bard = D @ ba
            baoa = ba @ oa
            rdoa = D @ oa
            oaoa = oa @ oa
            a = baba - bard * bard
            b = baba * rdoa - baoa * bard
            cc = baba * oaoa - baoa * baoa - r * r * baba
            h = b * b - a * cc
            ok = (h >= 0) & (a > 1e-9)
            if not ok.any():
                continue
            sq = np.sqrt(np.maximum(h, 0))
            t = (-b - sq) / np.where(a > 1e-9, a, 1)
            yv = baoa + t * bard
            body = ok & (yv > 0) & (yv < baba) & (t > 0)
            # 끝 둥근 마개 (이음매를 메운다)
            cap_t = np.full_like(t, np.inf)
            for P in (A, B):
                oc = O - P
                b2 = D @ oc
                c2 = oc @ oc - r * r
                h2 = b2 * b2 - c2
                t2 = -b2 - np.sqrt(np.maximum(h2, 0))
                cap_t = np.where((h2 >= 0) & (t2 > 0) & (t2 < cap_t), t2, cap_t)
            th = np.where(body, t, cap_t)
            win = th < t_best
            if not win.any():
                continue
            idx = np.nonzero(win)[0]
            tt = th[idx]
            P = O + D[idx] * tt[:, None]
            y = np.clip(((P - A) @ ba) / baba, 0, 1)
            q = A + y[:, None] * ba
            n = (P - q) / r
            shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
            s = s0 + y * np.sqrt(baba)
            rib = 0.88 + 0.12 * np.cos(2 * np.pi * s / self.rib)
            t_best[idx] = tt
            col[idx] = self.duct_bgr * (shade * rib)[:, None]

    def _render_boxes(self, O, D_all, t_all, col_all):
        light = np.array([0.3, 0.2, 0.93], np.float32)
        light /= np.linalg.norm(light)
        for c, half, yaw, bgr in self.boxes:
            corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * half[:2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            sl = self._ray_slice(corners @ rot.T + c[:2], 0.05)
            if sl.stop <= sl.start:
                continue
            D, t_best, col = D_all[sl], t_all[sl], col_all[sl]
            cyw, syw = np.cos(-yaw), np.sin(-yaw)
            R = np.array([[cyw, -syw, 0], [syw, cyw, 0], [0, 0, 1]], np.float32)
            o = R @ (O - np.array(c, np.float32))
            d = D @ R.T
            hs = np.array(half, np.float32)
            with np.errstate(divide='ignore', invalid='ignore'):
                inv = 1.0 / np.where(np.abs(d) < 1e-9, 1e-9, d)
                t1 = (-hs - o) * inv
                t2 = (hs - o) * inv
            tmin = np.minimum(t1, t2).max(1)
            tmax = np.maximum(t1, t2).min(1)
            hit = (tmax >= tmin) & (tmin > 0)
            win = hit & (tmin < t_best)
            if not win.any():
                continue
            idx = np.nonzero(win)[0]
            axis = np.argmax(np.minimum(t1, t2)[idx], 1)
            n_local = np.zeros((len(idx), 3), np.float32)
            n_local[np.arange(len(idx)), axis] = -np.sign(d[idx, axis])
            n = n_local @ R  # 상자 축 → map 축
            shade = 0.5 + 0.5 * np.clip(n @ light, 0, 1)
            t_best[idx] = tmin[idx]
            col[idx] = np.array(bgr, np.float32) * shade[:, None]


def camera_from_ost(ost_K, ost_D, calib_size, size, x, y, z, pitch_deg):
    """풀해상도 보정값(K)을 실제 영상 크기로 줄여 Camera 를 만든다."""
    K = np.array(ost_K, float).reshape(3, 3).copy()
    K[0] *= size[0] / calib_size[0]
    K[1] *= size[1] / calib_size[1]
    return Camera(size[0], size[1], K, np.array(ost_D, float), x, y, z, pitch_deg)
