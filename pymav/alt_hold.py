from pymavlink import mavutil
import time
import keyboard
import signal
import threading

# connection_string = 'COM15'
connection_string = "COM12"  # Replace with your port
baud_rate = 57600

barometer_altitude = 0.0
initial_pressure = (
    None  # Initialize as None to store first received pressure value dynamically
)
rho = 1.225  # Standard air density at sea level in kg/mÂ³
g = 9.81

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat received! Pixhawk is connected.")


def request_baro_data():
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # Request sensor data
        10,  # Frequency (Hz)
        1,
    )


request_baro_data()


def read_baro_data():
    global barometer_altitude, initial_pressure, rho, g
    while True:
        msg = master.recv_match(
            type=["SCALED_PRESSURE", "SCALED_PRESSURE2"], blocking=False
        )  # Non-blocking
        if msg:
            pressure = msg.press_abs  # Pressure in hPa
            if initial_pressure is None:
                initial_pressure = pressure  # Set reference pressure at startup
                print(f"Initial Pressure Recorded: {initial_pressure:.2f} hPa")
                continue
            barometer_altitude = (
                (initial_pressure - pressure) * 100 / (rho * g)
            )  # h = (P_ref - P) / (Ïg)
        time.sleep(0.1)  # Prevent CPU overuse


# Start Barometer Data Retrieval in a Separate Thread
baro_thread = threading.Thread(target=read_baro_data)
baro_thread.daemon = True
baro_thread.start()


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
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
    )
    print(f"Mode set to {mode}")
    time.sleep(2)


def land_drone():
    print("Landing initiated...")
    set_mode("LAND")


def disarm_drone():
    print("Disarming drone...")
    master.arducopter_disarm()
    print("Drone disarmed.")


def send_rc_override(throttle_pwm):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,
        1500,
        throttle_pwm,
        1500,
        0,
        0,
        0,
        0,
    )


def takeoff():
    print("Taking off...")
    count = 1
    for pwm in range(1200, 1650, 10):
        curr_alt = barometer_altitude  # Real-time updated altitude
        print(f"Current Altitude: {curr_alt:.2f}m")

        if curr_alt > 0.3:
            pwm -= 10 * count
            count += 1
        if curr_alt >= 1.6:
            set_mode("ALT_HOLD")
            send_rc_override(1510)
            break
        if get_mode() != "STABILIZE":
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

set_mode("STABILIZE")

print("Disabling safety switch...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
    0,
    220,
    0,
    0,
    0,
    0,
    0,
    0,
)
time.sleep(2)
print("Safety switch disabled.")

print("Arming drone...")
master.arducopter_arm()
master.motors_armed_wait()
print("Motors armed.")

takeoff()
time.sleep(30)

print("Landing...")
set_mode("LAND")
time.sleep(30)

print("Disarming...")
master.arducopter_disarm()
print("Drone disarmed.")
