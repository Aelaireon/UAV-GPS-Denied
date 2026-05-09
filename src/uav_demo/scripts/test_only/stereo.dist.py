import subprocess
import numpy as np
import cv2
import os

os.environ['DISPLAY'] = ':1001'

# ── USER SETTINGS ────────────────────────────────────────────────────────────
MARKER_REAL_SIZE_M = 0.253   # <-- Set this: printed marker width in metres (e.g. 0.15 = 15cm)
KNOWN_DISTANCE_M   = 1.0    # <-- Distance you'll stand from camera when pressing 'c' to calibrate
# ─────────────────────────────────────────────────────────────────────────────

focal_length_px = 565.8   # calibrated value, will be changed temporarily after pressing 'c'

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
        "--gain", "2.0",
        "--awb", "indoor",
        "--inline",
        "-o", "-"
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

proc_l = get_cam_proc(0)
proc_r = get_cam_proc(1)

aruco_params  = cv2.aruco.DetectorParameters()
aruco_dict6   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
detector6     = cv2.aruco.ArucoDetector(aruco_dict6, aruco_params)

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

def marker_pixel_width(corners):
    """Average width of marker in pixels using all 4 sides."""
    pts = corners[0]
    side_lengths = [
        np.linalg.norm(pts[i] - pts[(i+1) % 4])
        for i in range(4)
    ]
    return np.mean(side_lengths)

def estimate_distance(pixel_width):
    """Distance = (real_size * focal_length) / pixel_width"""
    if focal_length_px is None or pixel_width == 0:
        return None
    return (MARKER_REAL_SIZE_M * focal_length_px) / pixel_width

def draw_distance(frame, corners, ids):
    for i, corner in enumerate(corners):
        px_width = marker_pixel_width(corner)
        dist     = estimate_distance(px_width)
        pts      = corner[0].astype(int)

        # Green border
        cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

        # Label: ID + distance
        mid_x = int(np.mean(pts[:, 0]))
        mid_y = int(np.mean(pts[:, 1]))
        marker_id = ids[i][0] if ids is not None else "?"

        if dist is not None:
            label = f"ID:{marker_id}  {dist:.2f}m"
            color = (0, 255, 0)
        else:
            label = f"ID:{marker_id}  [press C to calib]"
            color = (0, 200, 255)

        cv2.putText(frame, label, (mid_x - 60, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

buffer_l = b""
buffer_r = b""

print("Controls:")
print("  C  — hold marker at exactly {:.1f}m from LEFT camera, then press C to calibrate focal length".format(KNOWN_DISTANCE_M))
print("  Q  — quit")

try:
    while True:
        frame_l, buffer_l = get_frame(proc_l, buffer_l)
        frame_r, buffer_r = get_frame(proc_r, buffer_r)

        frame_l = cv2.rotate(frame_l, cv2.ROTATE_180)
        frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

        corners_l, ids_l, _ = detector6.detectMarkers(frame_l)
        corners_r, ids_r, _ = detector6.detectMarkers(frame_r)

        if ids_l is not None:
            cv2.aruco.drawDetectedMarkers(frame_l, corners_l, ids_l)
            draw_distance(frame_l, corners_l, ids_l)

        if ids_r is not None:
            cv2.aruco.drawDetectedMarkers(frame_r, corners_r, ids_r)
            draw_distance(frame_r, corners_r, ids_r)

        # Status overlay
        status = f"Focal length: {focal_length_px:.1f}px" if focal_length_px else "Not calibrated — press C"
        cv2.putText(frame_l, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        stereo_view = np.hstack((frame_l, frame_r))
        cv2.imshow("Stereo ArUco - Distance", stereo_view)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('c'):
            # Calibrate focal length from left camera
            if ids_l is not None and len(corners_l) > 0:
                px_width       = marker_pixel_width(corners_l[0])
                focal_length_px = (px_width * KNOWN_DISTANCE_M) / MARKER_REAL_SIZE_M
                print(f"Calibrated! Focal length = {focal_length_px:.1f}px  (marker pixel width was {px_width:.1f}px)")
            else:
                print("No marker detected in LEFT camera — hold marker in view and try again")

finally:
    proc_l.terminate()
    proc_r.terminate()
    cv2.destroyAllWindows()
