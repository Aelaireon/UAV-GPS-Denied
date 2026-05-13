#!/usr/bin/env python3

import subprocess
import numpy as np
import cv2
import os

os.environ['DISPLAY'] = ':1001'

# ── Load stereo calibration ───────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
# CALIB_PATH  = os.path.join(SCRIPT_DIR, "stereo_run_003", "calib_stereo.npz")
# CALIB_PATH  = "/home/uav/UAV-GPS-Denied/install/uav_demo/lib/uav_demo/scripts/test_only/stereo_run_012/calib_stereo.npz"
CALIB_PATH  = "/home/uav/UAV-GPS-Denied/src/uav_demo/scripts/test_only/stereo_run_002/calib_stereo.npz"
# Measured: 5.02m  |  Actual: 2.565m (101 inches)
# DIST_SCALE = 2.565 / 5.02   # = 0.511
DIST_SCALE = 1.0

print("Loading stereo calibration from:", CALIB_PATH)
c = np.load(CALIB_PATH)

map_lx    = c['map_lx'];   map_ly = c['map_ly']
map_rx    = c['map_rx'];   map_ry = c['map_ry']
focal_px  = float(c['focal_px'])
baseline_m = float(c['baseline_m'])
Q         = c['Q']

print(f"  Focal length : {focal_px:.2f} px")
print(f"  Baseline     : {baseline_m*1000:.2f} mm")

# ── ArUco ─────────────────────────────────────────────────────────────────────
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ── Camera procs ──────────────────────────────────────────────────────────────
def get_cam_proc(camera_index):
    cmd = [
        "rpicam-vid",
        "--camera", str(camera_index),
        "--codec", "mjpeg",
        "-t", "0",
        "--width", "1280",          # 720p width
        "--height", "720",          # 720p height
        "--nopreview",
        "--framerate", "60",       # Target 60fps
        "--shutter", "10000",       # Shutter speed in microseconds (1/100s = 10,000us)
        "--gain", "5.0",
        "--awb", "indoor",
        "--denoise", "cdn_off",     # Recommended for high FPS to save CPU
        "--inline",
        "-o", "-"
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

def get_frame(proc, buffer):
    while True:
        buffer += proc.stdout.read(4096)
        a = buffer.find(b'\xff\xd8')
        b = buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg    = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame  = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame, buffer

def marker_center(corners):
    """Return (cx, cy) center of a detected marker."""
    pts = corners[0]
    return np.mean(pts, axis=0)

def stereo_distance(cx_l, cx_r):
    """
    distance = focal_px * baseline_m / disparity
    disparity = x_left - x_right  (must be positive; left image sees object more to the right)
    """
    disparity = cx_l - cx_r
    if disparity <= 0:
        return None
    return ((focal_px * baseline_m) / disparity) * DIST_SCALE

proc_l = get_cam_proc(0)
proc_r = get_cam_proc(1)
buf_l  = b""
buf_r  = b""

print("\nRunning — press Q to quit")
print("Green label = stereo distance  |  Yellow = single-camera fallback (marker only in one view)\n")

cv2.namedWindow("Stereo Distance", cv2.WINDOW_NORMAL)
# Optional: Set a default starting size so it's not a tiny square
cv2.resizeWindow("Stereo Distance", 1280, 480)

try:
    while True:
        frame_l, buf_l = get_frame(proc_l, buf_l)
        frame_r, buf_r = get_frame(proc_r, buf_r)

        # Rotate
        frame_l = cv2.rotate(frame_l, cv2.ROTATE_180)
        frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

        # Rectify using calibration maps
        rect_l = cv2.remap(frame_l, map_lx, map_ly, cv2.INTER_LINEAR)
        rect_r = cv2.remap(frame_r, map_rx, map_ry, cv2.INTER_LINEAR)

        # Detect markers in rectified frames
        corners_l, ids_l, _ = detector.detectMarkers(rect_l)
        corners_r, ids_r, _ = detector.detectMarkers(rect_r)

        # Draw detections
        if ids_l is not None:
            cv2.aruco.drawDetectedMarkers(rect_l, corners_l, ids_l)
        if ids_r is not None:
            cv2.aruco.drawDetectedMarkers(rect_r, corners_r, ids_r)

        # Build ID → corners lookup
        map_l = {ids_l[i][0]: corners_l[i] for i in range(len(ids_l))} if ids_l is not None else {}
        map_r = {ids_r[i][0]: corners_r[i] for i in range(len(ids_r))} if ids_r is not None else {}

        all_ids = set(map_l.keys()) | set(map_r.keys())

        for mid in all_ids:
            in_l = mid in map_l
            in_r = mid in map_r

            if in_l and in_r:
                # ── Stereo triangulation ──────────────────────────────────────
                cx_l, cy_l = marker_center(map_l[mid])
                cx_r, cy_r = marker_center(map_r[mid])
                dist = stereo_distance(cx_l, cx_r)

                if dist is not None:
                    label = f"ID:{mid}  {dist:.2f}m"
                    color = (0, 255, 0)
                    # Draw on both frames
                    for frame, cx, cy in [(rect_l, cx_l, cy_l), (rect_r, cx_r, cy_r)]:
                        cv2.putText(frame, label,
                                    (int(cx) - 50, int(cy) - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                        cv2.polylines(frame,
                                      [map_l[mid][0].astype(int) if frame is rect_l
                                       else map_r[mid][0].astype(int)],
                                      True, color, 2)
                else:
                    # Negative disparity — cameras may be swapped
                    for frame, corners in [(rect_l, map_l[mid]), (rect_r, map_r[mid])]:
                        cx, cy = marker_center(corners)
                        cv2.putText(frame, f"ID:{mid} [check cam order]",
                                    (int(cx) - 60, int(cy) - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)

            elif in_l:
                # Only left camera — no stereo, show warning
                cx, cy = marker_center(map_l[mid])
                cv2.putText(rect_l, f"ID:{mid} [L only]",
                            (int(cx) - 50, int(cy) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            elif in_r:
                # Only right camera
                cx, cy = marker_center(map_r[mid])
                cv2.putText(rect_r, f"ID:{mid} [R only]",
                            (int(cx) - 50, int(cy) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # Draw a horizontal epipolar line to verify rectification
        for y in range(0, rect_l.shape[0], 60):
            cv2.line(rect_l, (0, y), (rect_l.shape[1], y), (40, 40, 40), 1)
            cv2.line(rect_r, (0, y), (rect_r.shape[1], y), (40, 40, 40), 1)

        stereo_view = np.hstack((rect_l, rect_r))
        cv2.imshow("Stereo Distance", stereo_view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    proc_l.terminate()
    proc_r.terminate()
    cv2.destroyAllWindows()