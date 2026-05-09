import subprocess
import numpy as np
import cv2
import os

# 1. Setup Display for your dummy driver
os.environ['DISPLAY'] = ':1001'
#os.environ['DISPLAY'] = ':1'

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
        #"--shutter", "5000",  # 5ms shutter to freeze motion
        "--gain", "2.0",       # Boost gain to compensate for fast shutter
        "--awb", "indoor", # Fix white balance for consistent marker detection
        "--inline",
        "-o", "-"
        # Inside get_cam_proc, add:
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)


# 2. Start both camera processes
proc_l = get_cam_proc(0)
proc_r = get_cam_proc(1)

# 3. Setup ArUco
#aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_dict5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
aruco_dict6 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
detector5 = cv2.aruco.ArucoDetector(aruco_dict5, aruco_params)
detector6 = cv2.aruco.ArucoDetector(aruco_dict6, aruco_params)

def get_frame(proc, buffer):
    # Standard MJPEG stream parsing logic
    # (Look for start 0xff 0xd8 and end 0xff 0xd9)
    while True:
        buffer += proc.stdout.read(4096)
        a = buffer.find(b'\xff\xd8')
        b = buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame, buffer

buffer_l = b""
buffer_r = b""

try:
    while True:
        frame_l, buffer_l = get_frame(proc_l, buffer_l)
        frame_r, buffer_r = get_frame(proc_r, buffer_r)
        
        #frame_l = cv2.flip(frame_l, 0)
        #frame_r = cv2.flip(frame_r, 0)
        frame_l = cv2.rotate(frame_l, cv2.ROTATE_180)
        frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)

        # ArUco Detection
        corners_l, ids_l, _ = detector6.detectMarkers(frame_l)
        corners_r, ids_r, _ = detector6.detectMarkers(frame_r)
        #if ids_l is not None and corners_l is not None:
        #    corners_l, ids_l = detector5.detectMarkers(frame_l)
        #if ids_r is not None and corners_r is not None:
        #    corners_r, ids_r = detector5.detectMarkers(frame_r)

        #if ids_l is not None: cv2.aruco.drawDetectedMarkers(frame_l, corners_l, ids_l)
        #if ids_r is not None: cv2.aruco.drawDetectedMarkers(frame_r, corners_r, ids_r)
        if ids_l is not None:
            cv2.aruco.drawDetectedMarkers(frame_l, corners_l, ids_l)
            for corner in corners_l:
                pts = corner[0].astype(int)
                cv2.polylines(frame_l, [pts], True, (0, 255, 0), 3)  # thicker green border

        if ids_r is not None:
            cv2.aruco.drawDetectedMarkers(frame_r, corners_r, ids_r)
            for corner in corners_r:
                pts = corner[0].astype(int)
                cv2.polylines(frame_r, [pts], True, (0, 255, 0), 3)

        # Show Results
        stereo_view = np.hstack((frame_l, frame_r))
        cv2.imshow("Stereo ArUco", stereo_view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    proc_l.terminate()
    proc_r.terminate()
    cv2.destroyAllWindows()

