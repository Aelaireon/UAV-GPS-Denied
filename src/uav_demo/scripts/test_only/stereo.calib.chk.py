import subprocess
import numpy as np
import cv2
import os
import time

os.environ['DISPLAY'] = ':1001'

# ── BOARD SETTINGS ────────────────────────────────────────────────────────────
INNER_COLS  = 10      # inner corners horizontally (squares - 1)
INNER_ROWS  = 7       # inner corners vertically   (squares - 1)
CHECKER_MM  = 88.65    # physical square size in mm (measure on screen to confirm!)
# ─────────────────────────────────────────────────────────────────────────────

FRAMES_NEEDED = 20
SAVE_DIR      = os.path.dirname(os.path.abspath(__file__))

# Termination criteria for corner refinement
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D object points for one board view (flat, z=0)
objp = np.zeros((INNER_ROWS * INNER_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:INNER_COLS, 0:INNER_ROWS].T.reshape(-1, 2)
objp *= (CHECKER_MM / 1000.0)   # convert to metres

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

def detect_corners(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, (INNER_COLS, INNER_ROWS), None)
    if found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return found, corners, gray

def calibrate_camera(cam_index, label, save_dir):
    print(f"\n{'='*50}")
    print(f"  Calibrating {label} camera (index {cam_index})")
    print(f"  Board: {INNER_COLS}x{INNER_ROWS} inner corners, {CHECKER_MM}mm squares")
    print(f"  Need {FRAMES_NEEDED} good captures")
    print(f"  Auto-captures every 1s when board is detected")
    print(f"  Q = quit")
    print(f"{'='*50}\n")

    proc   = get_cam_proc(cam_index)
    buffer = b""

    obj_points  = []
    img_points  = []
    image_size  = None
    captured    = 0
    last_msg    = ""
    last_capture_time = 0

    try:
        while captured < FRAMES_NEEDED:
            frame, buffer = get_frame(proc, buffer)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            found, corners, gray = detect_corners(frame)

            if image_size is None:
                image_size = gray.shape[::-1]

            vis = frame.copy()

            now          = time.time()
            time_since   = now - last_capture_time
            countdown    = max(0.0, 1.0 - time_since)

            if found:
                cv2.drawChessboardCorners(vis, (INNER_COLS, INNER_ROWS), corners, found)
                status_text  = f"GOOD — capturing in {countdown:.1f}s"
                status_color = (0, 255, 0)
            else:
                status_text  = "Board not detected — reposition"
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
                print("Quitting.")
                return None, None, None

            # Auto capture every 1 second when board is detected
            if found and time_since >= 1.0:
                obj_points.append(objp)
                img_points.append(corners)
                captured += 1
                last_capture_time = now
                img_path = os.path.join(save_dir, f"frame_{captured:03d}.jpg")
                cv2.imwrite(img_path, frame)
                last_msg = f"Auto-captured #{captured}!"
                print(f"  Captured frame {captured}/{FRAMES_NEEDED} -> {img_path}")

    finally:
        proc.terminate()
        cv2.destroyAllWindows()

    print(f"\nRunning calibration for {label} camera...")
    rms, K, D, _, _ = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)
    print(f"  RMS reprojection error: {rms:.4f}  (good if < 1.0)")
    return rms, K, D

# ── Main ──────────────────────────────────────────────────────────────────────
print("Checkerboard Camera Calibration")
print(f"Board: {INNER_COLS}x{INNER_ROWS} inner corners, {CHECKER_MM}mm squares")
print("Calibrating LEFT camera first, then RIGHT.\n")

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
