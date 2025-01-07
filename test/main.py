from pymavlink import mavutil
import time
import rospy
from nav_msgs.msg import Odometry
import math

def calculate_next_coordinates(time_interval, displacement, current_lat, current_lon, current_alt):
    # Constants
    R = 6378137  # Radius of Earth in meters

    # Displacement in meters
    d_lat = displacement[0]
    d_lon = displacement[1]
    d_alt = displacement[2]  # Displacement in altitude

    # Convert current coordinates to radians
    current_lat_rad = math.radians(current_lat)
    current_lon_rad = math.radians(current_lon)

    # Calculate new latitude
    new_lat_rad = current_lat_rad + (d_lat / R)
    new_lat = math.degrees(new_lat_rad)

    # Calculate new longitude
    new_lon_rad = current_lon_rad + (d_lon / (R * math.cos(current_lat_rad)))
    new_lon = math.degrees(new_lon_rad)

    # Calculate new altitude
    new_alt = current_alt + d_alt

    return new_lat, new_lon, new_alt


def transform_coordinates(bbox, angle):
    # bbox: (w, h, x, y) where (x, y) is the center of the bounding box
    # angle: orientation angle in radians

    w, h, x, y = bbox

    # Convert (x, y) to coordinates with image center as origin
    img_center_x = 0.5  # Assuming normalized coordinates (0 to 1)
    img_center_y = 0.5  # Assuming normalized coordinates (0 to 1)
    x_centered = x - img_center_x
    y_centered = y - img_center_y

    # Apply rotation matrix
    new_x = x_centered * math.cos(angle) - y_centered * math.sin(angle)
    new_y = x_centered * math.sin(angle) + y_centered * math.cos(angle)

    return new_x, new_y


def check_gnss_status(master):
    # 確認是否有GNSS訊號
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
    # 取得經緯度座標
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
    # 手動寫入GNSS座標給飛控板
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

# 定義回調函數，當接收到來自 /vins_estimator/odometry 話題的消息時，這個函數會被自動調用，msg 是接收到的消息。
def odometry_callback(msg):
    # 提取位置資訊
    position = msg.pose.pose.position
    x = position.x
    y = position.y
    z = position.z

    # 提取時間戳（以 ROS 時間為單位）。
    timestamp = msg.header.stamp

    # 在此處計算單位時間的路徑位移
    # 例如，您可以將當前位置與前一位置進行比較，計算位移量
    # 並根據時間差計算速度或其他所需的數據

    # 將時間戳和位置資訊以可讀格式輸出到 ROS 的日誌系統中，rospy.loginfo 會在終端顯示資訊。
    rospy.loginfo(f"Time: {timestamp.to_sec()}s, Position -> x: {x}, y: {y}, z: {z}")

    

def listener():
    # 初始化一個名為 vins_odometry_listener 的 ROS 節點，anonymous=True 確保每個節點名稱是唯一的（會自動添加隨機後綴）。
    rospy.init_node('vins_odometry_listener', anonymous=True)

    # 訂閱 /vins_estimator/odometry 話題，並將接收到的消息傳遞給 odometry_callback 函數進行處理。
    rospy.Subscriber('/vins_estimator/odometry', Odometry, odometry_callback)

    # 保持節點運行狀態，等待話題消息的到來。如果沒有這行代碼，節點會在初始化後立即退出。
    rospy.spin()


#連接到飛控板
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=115200)

if check_gnss_status(master):
    lat, lon, alt = get_gnss_position(master)

    # 呼叫 listener 函數，啟動 ROS 節點並開始訂閱話題。
    listener()
else:
    print("GNSS signal not available. Writing manual GNSS position.")
    #manual_latitude = 37.7749
    #manual_longitude = -122.4194
    #manual_altitude = 10  # meters  
    #write_gnss_position(master, manual_latitude, manual_longitude, manual_altitude)
