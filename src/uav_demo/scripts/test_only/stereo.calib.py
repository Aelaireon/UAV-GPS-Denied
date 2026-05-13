#!/usr/bin/env python3

import subprocess
import numpy as np
import cv2
import os
import time

os.environ['DISPLAY'] = ':1001'

# ── BOARD SETTINGS ────────────────────────────────────────────────────────────
COLS         = 11
ROWS         = 8
CHECKER_MM   = 97.19
MARKER_MM    = 67.50
# ─────────────────────────────────────────────────────────────────────────────

FRAMES_NEEDED = 20
SAVE_DIR      = os.path.dirname(os.path.abspath(__file__))

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
board      = cv2.aruco.CharucoBoard(
                (COLS, ROWS),
                CHECKER_MM / 1000.0,
                MARKER_MM  / 1000.0,
                aruco_dict
             )
detector   = cv2.aruco.CharucoDetector(board)

# ── Run folder ────────────────────────────────────────────────────────────────
def get_next_run_dir(base):
    i = 1
    while os.path.exists(os.path.join(base, f"calib_final_{i:03d}")):
        i += 1
    run_dir = os.path.join(base, f"calib_final_{i:03d}")
    os.makedirs(os.path.join(run_dir, "left"),  exist_ok=True)
    os.makedirs(os.path.join(run_dir, "right"), exist_ok=True)
    print(f"Saving images to: {run_dir}")
    return run_dir

RUN_DIR = get_next_run_dir(SAVE_DIR)

def get_cam_proc(camera_index):
    cmd = [
        "rpicam-vid",
        "--camera", str(camera_index),
        "--codec", "mjpeg",
        "-t", "0",
        "--width", "640",
        "--height", "480",
        "--nopreview",
        "--framerate", "30",
        "--shutter", "15000",
        "--gain", "3.0",
        "--autofocus-mode", "manual",
        "--lens-position", "0.0",
        "--awb", "indoor",
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

def detect_charuco(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    return charuco_corners, charuco_ids, marker_corners, marker_ids, gray

def draw_detections(frame, charuco_corners, charuco_ids, marker_corners, marker_ids):
    out = frame.copy()
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(out, marker_corners, marker_ids)
    if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 4:
        cv2.aruco.drawDetectedCornersCharuco(out, charuco_corners, charuco_ids, (0, 255, 0))
    return out

def run_calibration(all_corners, all_ids, image_size):
    obj_points = []
    img_points = []
    for corners, ids in zip(all_corners, all_ids):
        obj_pts, img_pts = board.matchImagePoints(corners, ids)
        obj_points.append(obj_pts)
        img_points.append(img_pts)
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None
    )
    return ret, camera_matrix, dist_coeffs

def calibrate_camera(cam_index, label, save_dir):
    print(f"\n{'='*50}")
    print(f"  Calibrating {label} camera (index {cam_index})")
    print(f"  Need {FRAMES_NEEDED} good captures")
    print(f"  Auto-captures every 1s when board detected")
    print(f"  Q = quit")
    print(f"{'='*50}\n")

    proc              = get_cam_proc(cam_index)
    buffer            = b""
    all_corners       = []
    all_ids           = []
    image_size        = None
    captured          = 0
    last_msg          = ""
    last_capture_time = 0

    try:
        while captured < FRAMES_NEEDED:
            frame, buffer = get_frame(proc, buffer)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            charuco_corners, charuco_ids, marker_corners, marker_ids, gray = detect_charuco(frame)

            if image_size is None:
                image_size = gray.shape[::-1]

            vis        = draw_detections(frame, charuco_corners, charuco_ids, marker_corners, marker_ids)
            now        = time.time()
            time_since = now - last_capture_time
            countdown  = max(0.0, 1.0 - time_since)
            n_corners  = len(charuco_corners) if charuco_corners is not None else 0
            good       = n_corners >= 6

            if good:
                status_text  = f"GOOD — capturing in {countdown:.1f}s"
                status_color = (0, 255, 0)
            else:
                status_text  = f"Corners: {n_corners}  [need more — reposition]"
                status_color = (0, 100, 255)

            cv2.putText(vis, f"{label} | Captured: {captured}/{FRAMES_NEEDED}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, status_text, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            cv2.putText(vis, "Q=quit", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            if last_msg:
                cv2.putText(vis, last_msg, (10, 460),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            cv2.imshow(f"Calibration - {label}", vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("Quitting calibration.")
                return None, None, None

            if good and time_since >= 1.0:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)
                captured += 1
                last_capture_time = now
                img_path = os.path.join(save_dir, f"frame_{captured:03d}.jpg")
                cv2.imwrite(img_path, frame)
                last_msg = f"Auto-captured #{captured}!  ({n_corners} corners)"
                print(f"  Captured frame {captured}/{FRAMES_NEEDED}  ({n_corners} corners)")

    finally:
        proc.terminate()
        cv2.destroyAllWindows()

    print(f"\nRunning calibration for {label} camera...")
    rms, camera_matrix, dist_coeffs = run_calibration(all_corners, all_ids, image_size)
    print(f"  RMS reprojection error: {rms:.4f}  (good if < 1.0)")
    return rms, camera_matrix, dist_coeffs

# ── Main ──────────────────────────────────────────────────────────────────────
print("ChArUco Camera Calibration")
print(f"Board: {COLS}x{ROWS} squares, {CHECKER_MM}mm checker, {MARKER_MM}mm marker, DICT_6X6\n")

rms_l, K_l, D_l = calibrate_camera(0, "LEFT",  os.path.join(RUN_DIR, "left"))
if K_l is None:
    print("Left calibration aborted.")
    exit()

rms_r, K_r, D_r = calibrate_camera(1, "RIGHT", os.path.join(RUN_DIR, "right"))
if K_r is None:
    print("Right calibration aborted.")
    exit()

out_path = os.path.join(RUN_DIR, "calib_intrinsics.npz")
np.savez(out_path,
         K_l=K_l, D_l=D_l, rms_l=rms_l,
         K_r=K_r, D_r=D_r, rms_r=rms_r)

print(f"\n{'='*50}")
print(f"Calibration complete!")
print(f"  Left  RMS : {rms_l:.4f}")
print(f"  Right RMS : {rms_r:.4f}")
print(f"  Saved to  : {out_path}")
print(f"{'='*50}")
print("\nNext step: stereo calibration (stereo.stereo_calib.py)")