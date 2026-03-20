from pymavlink import mavutil
import time
import keyboard
import signal
import threading

#connection_string = 'COM15'
connection_string = 'COM6'  # Replace with your port
baud_rate = 57600

rangefinder_distance = 0.0

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat received! Pixhawk is connected.")

def request_rangefinder_data():
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,  # Request rangefinder data
        10,  # Frequency (Hz)
        1
    )
request_rangefinder_data()

def read_rangefinder_data():
    global rangefinder_distance
    while True:
        msg = master.recv_match(type="RANGEFINDER", blocking=False)  # Non-blocking
        if msg:
            rangefinder_distance = msg.distance
        time.sleep(0.1)  # Prevent CPU overuse

# Start Rangefinder Data Retrieval in a Separate Thread
rangefinder_thread = threading.Thread(target=read_rangefinder_data)
rangefinder_thread.daemon = True
rangefinder_thread.start()

def get_mode():
    try:
        mode = master.flightmode
        print(f" - Current mode: {mode}")
        return mode
    except Exception as e:
        print(f"Error getting mode: {e}")
        return None

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")
    time.sleep(2)

def clear_rc_override():
    """Release RC override so autopilot has full control"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0,  # 0 = release override on all channels
        0, 0, 0, 0
    )
    time.sleep(0.1)

def land_drone():
    print("Landing initiated...")
    clear_rc_override()        # ← Release throttle override first
    time.sleep(0.3)            # ← Brief pause to let autopilot resume control
    set_mode("LAND")

def disarm_drone():
    print("Disarming drone...")
    master.arducopter_disarm()
    print("Drone disarmed.")

def send_rc_override(throttle_pwm):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, throttle_pwm, 1500,
        0, 0, 0, 0
    )

def takeoff():
    print("Taking off...")
    count = 1
    for pwm in range(1400, 1850, 10):
        curr_alt = rangefinder_distance  # Real-time updated altitude
        print(f"Current Altitude: {curr_alt:.2f}m")
        
        if curr_alt > 0.35:
            pwm -= 10 * count
            count += 1
        if curr_alt >= 0.4:
            set_mode("FLOWHOLD")
            send_rc_override(1550)
            break
        if get_mode() != 'FLOWHOLD':
            set_mode("LAND")
            break
        if keyboard.is_pressed("l"):
            break
        send_rc_override(pwm)
        print(f"Throttle PWM: {pwm}")
        time.sleep(0.3)

def emergency_shutdown():
    print("\n[CTRL+C DETECTED] Emergency landing and disarm initiated!")
    land_drone()
    disarm_drone()
    print("Exiting safely.")
    exit(0)

keyboard.add_hotkey("l", land_drone)
keyboard.add_hotkey("q", disarm_drone)
signal.signal(signal.SIGINT, lambda sig, frame: emergency_shutdown())

set_mode("FLOWHOLD")

print("Disabling safety switch...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
    0, 220, 0, 0, 0, 0, 0, 0
)
time.sleep(2)
print("Safety switch disabled.")

print("Arming drone...")
master.arducopter_arm()
master.motors_armed_wait()
print("Motors armed.")

takeoff()
# if rangefinder_distance >= 1.2:
#             set_mode("FLOWHOLD")
#             send_rc_override(1550)
time.sleep(300)
land_drone()
time.sleep(30)

print("Disarming...")
master.arducopter_disarm()
print("Drone disarmed.")

