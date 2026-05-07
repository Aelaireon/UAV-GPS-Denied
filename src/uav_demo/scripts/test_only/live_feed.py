import os
import subprocess

env = os.environ.copy()
env['DISPLAY'] = ':1001'

cam = subprocess.Popen([
    'rpicam-vid',
    '--codec', 'mjpeg',
    '-t', '0',
    '--width', '640',
    '--height', '480',
    '--nopreview',        # ← stop rpicam trying to open its own window
    '-o', '-'
], stdout=subprocess.PIPE)

mpv = subprocess.Popen([
    'mpv',
    '--no-correct-pts',
    '--container-fps-override=30',   # ← match actual camera fps
    '--vo=x11',
    '--profile=low-latency',
    '--cache=no',
    '--demuxer-lavf-analyzeduration=0.1',
    '--demuxer-lavf-probesize=32',
    '-'
], stdin=cam.stdout, env=env)

cam.wait()
mpv.wait()
