from pymavlink import mavutil
import sys

connection_string = 'COM10' # or /dev/ttyACM0, NOT ACM1, ACM1 is purely logging stream, not sensor stream
baud_rate = 115200

print(f"Connecting to {connection_string} at {baud_rate}...")

try:
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
except Exception as e:
    print(f"Failed to connect: {e}")
    sys.exit()

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from System {master.target_system} Component {master.target_component}")

master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 10, 1
)

print("Rangefinder stream: ")

try:
    while True:
        msg = master.recv_match(type='RANGEFINDER', blocking=True, timeout=1.0)
        if msg:
            print(f"Distance: {msg.distance:.2f}m | Voltage: {msg.voltage:10f}V")
        else:
            print("No RANGEFINDER packet received. Is the sensor plugged into the I2C/Serial port?")
except KeyboardInterrupt:
    print("\nExiting...")
