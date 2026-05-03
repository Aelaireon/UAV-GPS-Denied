from pymavlink import mavutil
import time

# Use source_system=255 (companion computer), NOT 1 (autopilot)
master = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=57600,
    source_system=255,
    source_component=191  # MAV_COMP_ID_ONBOARD_COMPUTER
)

master.wait_heartbeat()
print("Heartbeat from system %u component %u" % (
    master.target_system, master.target_component))

def send_rangefinder_data(distance_cm):
    master.mav.distance_sensor_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,  # time_boot_ms — real uptime
        10,           # min_distance (cm)
        600,          # max_distance (cm)
        distance_cm,  # current_distance (cm)
        0,            # type: MAV_DISTANCE_SENSOR_LASER (0) — NOT the RNGFND_TYPE param
        0,            # id: must match RNGFND1 sensor index (0-based)
        25,           # orientation: MAV_SENSOR_ROTATION_PITCH_270 (downward)
        255           # signal_quality: 255 = unknown, 0 = NO SIGNAL (avoid 0!)
    )

while True:
    distance = 150  # cm — replace with real sensor read
    send_rangefinder_data(distance)
    print(f"Sent: {distance} cm")
    time.sleep(0.1)  # 10 Hz