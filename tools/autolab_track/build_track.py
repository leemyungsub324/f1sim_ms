#!/usr/bin/env python3
"""autolab_track.track.yaml → 시뮬레이터용 2D 점유맵과 확인용 그림을 만든다.

    python3 tools/autolab_track/build_track.py

만드는 파일 (f1tenth_gym_ros/maps/):
- autolab_track.png, autolab_track.yaml : f1sim C++ 코어와 map_server 가 읽는 2D 점유맵.
  라이다 높이에서 본 덕트·장애물·배경 물체만 들어간다. 노란 테이프는 라이다에 안 보이므로 넣지 않는다.

확인용 그림 (tools/autolab_track/):
- autolab_track_floor.png : 바닥 텍스처 (1 cm/px). 카메라 노드는 실행할 때 직접 만든다
- autolab_track_preview.png : 위에서 본 전체 그림 (점유 + 바닥 + 시작 자세)

맵을 고친 뒤에는 워크스페이스를 다시 빌드한다 (symlink-install 이면 생략 가능):
    colcon build --symlink-install --base-paths f1tenth_gym_ros
"""
import argparse
import os
import sys

import cv2
import numpy as np
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'f1tenth_gym_ros'))
from f1tenth_gym_ros.track_scene import floor_texture, load_track, occupancy_image  # noqa: E402

MAPS = os.path.join(ROOT, 'f1tenth_gym_ros', 'maps')
HERE = os.path.dirname(os.path.abspath(__file__))


def write_map_yaml(path, image, grid):
    meta = {'image': image, 'mode': 'trinary', 'resolution': grid.res,
            'origin': [float(v) for v in grid.origin], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.25}
    with open(path, 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default=os.path.join(MAPS, 'autolab_track.track.yaml'))
    ap.add_argument('--res', type=float, default=0.025, help='점유맵 해상도 (m/px)')
    ap.add_argument('--scan-height', type=float, default=0.11, help='라이다 높이 (m)')
    args = ap.parse_args()

    track = load_track(args.track)
    name = track['name']

    occ, grid = occupancy_image(track, args.res, args.scan_height)
    cv2.imwrite(os.path.join(MAPS, name + '.png'), occ)
    write_map_yaml(os.path.join(MAPS, name + '.yaml'), name + '.png', grid)

    tex, _ = floor_texture(track, 0.01)
    cv2.imwrite(os.path.join(HERE, name + '_floor.png'), tex)

    prev = cv2.resize(tex, (grid.w * 2, grid.h * 2), interpolation=cv2.INTER_AREA)
    occ2 = cv2.resize(occ, (grid.w * 2, grid.h * 2), interpolation=cv2.INTER_NEAREST)
    prev[occ2 < 128] = (prev[occ2 < 128] * 0.3).astype(np.uint8)
    s = track['start']
    p = ((s['x'] - grid.x0) / grid.res * 2, (grid.y1 - s['y']) / grid.res * 2)
    q = (p[0] + 40 * np.cos(s['theta']), p[1] - 40 * np.sin(s['theta']))
    cv2.arrowedLine(prev, tuple(int(v) for v in p), tuple(int(v) for v in q), (0, 0, 255), 3, tipLength=0.3)
    cv2.imwrite(os.path.join(HERE, name + '_preview.png'), prev)

    print(f'{name}: {grid.w}x{grid.h} px @ {grid.res} m, origin {grid.origin}')
    print(f"시작 자세: sx:={s['x']} sy:={s['y']} stheta:={s['theta']}")


if __name__ == '__main__':
    main()
