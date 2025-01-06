from pymavlink import mavutil
import time

def connect_to_flight_controller(connection_string):
    """Connect to the flight controller using MAVLink."""
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Heartbeat received from flight controller")
    return master

def check_gnss_status(master):
    """Check if the flight controller is receiving GNSS signal."""
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1)
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if not msg:
        print("No GPS_RAW_INT message received.")
        return False
    print(f"GNSS fix type: {msg.fix_type}, Satellites visible: {msg.satellites_visible}")
    if msg.fix_type >= 3:  # 3D Fix
        print("GNSS signal is available.")
        return True
    else:
        print("GNSS signal is not available.")
        return False

def get_gnss_position(master):
    """Get GNSS position from the flight controller."""
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg:
        lat = msg.lat / 1e7  # Convert to decimal degrees
        lon = msg.lon / 1e7  # Convert to decimal degrees
        alt = msg.alt / 1e3  # Convert to meters
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} meters")
        return lat, lon, alt
    else:
        print("Failed to receive GNSS position.")
        return None, None, None

def write_gnss_position(master, latitude, longitude, altitude):
    """Manually write GNSS position to the flight controller."""
    lat = int(latitude * 1e7)  # Convert to MAVLink format
    lon = int(longitude * 1e7)  # Convert to MAVLink format
    alt = int(altitude * 1e3)  # Convert to MAVLink format

    master.mav.gps_input_send(
        time_usec=int(time.time() * 1e6),
        gps_id=0,
        ignore_flags=0,
        time_week_ms=0,
        time_week=0,
        fix_type=3,  # 3D Fix
        lat=lat,
        lon=lon,
        alt=alt,
        hdop=1,
        vdop=1,
        vn=0,
        ve=0,
        vd=0,
        speed_accuracy=0,
        horiz_accuracy=0,
        vert_accuracy=0,
        satellites_visible=10
    )
    print("GNSS position written to flight controller.")

if __name__ == "__main__":
    # Replace with the appropriate connection string for your setup
    connection_string = "serial:/dev/ttyS0:57600"

    # Connect to the flight controller
    master = connect_to_flight_controller(connection_string)

    # Check GNSS status and retrieve position
    if check_gnss_status(master):
        lat, lon, alt = get_gnss_position(master)
    else:
        print("GNSS signal not available. Writing manual GNSS position.")
        # Replace with the desired manual GNSS position
        manual_latitude = 37.7749
        manual_longitude = -122.4194
        manual_altitude = 10  # meters
        write_gnss_position(master, manual_latitude, manual_longitude, manual_altitude)
