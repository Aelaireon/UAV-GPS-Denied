#!/usr/bin/env python3
import subprocess
import numpy as np
import cv2
import os

def main():
    os.environ['DISPLAY'] = ':1001'

    cmd = [
        "rpicam-vid",
        "--codec", "mjpeg",
        "-t", "0",
        "--width", "640",
        "--height", "480",
        "--nopreview",
        "--framerate", "30",
        "-o", "-"
    ]

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    print("Live feed started. Press 'q' to quit.")

    buf = b""

    try:
        while True:
            # Read chunk from rpicam-vid stdout
            chunk = proc.stdout.read(4096)
            if not chunk:
                break
            buf += chunk

            # Find JPEG start (FFD8) and end (FFD9) markers
            start = buf.find(b'\xff\xd8')
            end = buf.find(b'\xff\xd9')

            if start != -1 and end != -1 and end > start:
                # Extract one complete JPEG frame
                jpg = buf[start:end + 2]
                buf = buf[end + 2:]  # keep remainder for next frame

                # Decode JPEG → numpy array → display
                frame = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                
                frame = cv2.rotate(frame, cv2.ROTATE_180)

                if frame is not None:
                    cv2.imshow("UAV Camera Feed", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error occured {e}")
    finally:
        proc.terminate()
        proc.wait()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
