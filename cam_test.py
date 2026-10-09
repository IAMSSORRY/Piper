#!/usr/bin/env python3
"""카메라 연결 테스트 — 여러 소스를 나란히 띄우고 해상도·fps 표시.

  python3 cam_test.py                         # config.yaml cameras (top, wrist)
  python3 cam_test.py 2 /dev/v4l/by-path/...    # 번호·장치 경로·URL 직접 지정

키: s = 스냅샷 저장(snapshots/), r = 초점 최고값 리셋, q / Esc = 종료
초점: 노란 사각형(화면 가운데) 기준 FOCUS 숫자가 최대가 되게 렌즈를 돌린다 (초록 = 최고값 근처)
"""
import os
import sys
import time

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
LABELS = {}
TILE_H = 540   # 화면 타일 높이 (모니터 작으면 줄이기)


def name_of(src):
    if isinstance(src, str) and src.startswith("/dev/"):
        for k, v in LABELS.items():
            if v == src:
                return k
        return os.path.basename(os.path.realpath(src))
    if isinstance(src, int):
        p = f"/sys/class/video4linux/video{src}/name"
        if os.path.exists(p):
            return f"video{src}: {open(p).read().strip()}"
        return f"video{src}"
    return src


def open_src(src):
    v4l = isinstance(src, int) or src.startswith("/dev/")
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if v4l else cv2.VideoCapture(src)
    if v4l:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def main():
    args = sys.argv[1:]
    if args:
        srcs = [int(a) if a.isdigit() else a for a in args]
    else:
        cams = yaml.safe_load(open(os.path.join(HERE, "config.yaml"), encoding="utf-8"))["cameras"]
        LABELS.update({"top": cams["top"], "wrist": cams["wrist"]})
        srcs = [cams["top"], cams["wrist"]]
    caps = []
    for s in srcs:
        cap = open_src(s)
        ok = cap.isOpened() and cap.read()[0]
        print(f"{'OK ' if ok else 'FAIL'} {name_of(s)}")
        if ok:
            caps.append((s, cap, [time.monotonic(), 0, 0.0, None, 0.0]))
    if not caps:
        sys.exit("열린 카메라가 없습니다")

    os.makedirs(os.path.join(HERE, "snapshots"), exist_ok=True)
    while True:
        tiles = []
        for s, cap, st in caps:
            ok, f = cap.read()
            if not ok:
                f = np.zeros((TILE_H, 640, 3), np.uint8)
                cv2.putText(f, "NO FRAME", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            st[1] += 1
            if time.monotonic() - st[0] >= 1.0:
                st[2] = st[1] / (time.monotonic() - st[0])
                st[0], st[1] = time.monotonic(), 0
            h, w = f.shape[:2]
            st[3] = f
            # 초점 지표: 가운데 1/3 영역 라플라시안 분산 (클수록 선명). 최고값도 같이 표시
            c = cv2.cvtColor(f[h // 3:2 * h // 3, w // 3:2 * w // 3], cv2.COLOR_BGR2GRAY)
            sharp = cv2.Laplacian(c, cv2.CV_64F).var()
            st[4] = max(st[4], sharp)
            t = cv2.resize(f, (int(w * TILE_H / h), TILE_H))
            th, tw = t.shape[:2]
            cv2.rectangle(t, (tw // 3, th // 3), (2 * tw // 3, 2 * th // 3), (0, 255, 255), 1)
            cv2.putText(t, f"{name_of(s)}  {w}x{h}  {st[2]:.1f}fps", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            col = (0, 255, 0) if sharp >= 0.9 * st[4] else (0, 165, 255)
            cv2.putText(t, f"FOCUS {sharp:.0f}  (max {st[4]:.0f})", (8, th - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
            tiles.append(t)
        cv2.imshow("cam_test (s=save, r=reset focus max, q=quit)", np.hstack(tiles))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            for _, _, st in caps:
                st[4] = 0.0
        if k == ord("s"):
            ts = time.strftime("%H%M%S")
            for s, cap, st in caps:
                tag = name_of(s) if isinstance(s, str) and s.startswith("/dev/") else (f"video{s}" if isinstance(s, int) else "url")
                p = os.path.join(HERE, "snapshots", f"{ts}_{tag}.jpg")
                cv2.imwrite(p, st[3])
                print("저장", p)
    for _, cap, _ in caps:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
