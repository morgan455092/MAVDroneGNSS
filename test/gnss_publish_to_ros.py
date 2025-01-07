import rospy
from sensor_msgs.msg import NavSatFix
from pymavlink import mavutil
import math
import twd97

def publish_gnss():
    # Initialize ROS node
    rospy.init_node('gnss_publisher', anonymous=True)
    pub = rospy.Publisher('gnss_data', NavSatFix, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Connect to the flight controller
    mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)

    while not rospy.is_shutdown():
        # Wait for a GPS_RAW_INT message
        msg = mavlink_connection.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            # Create a NavSatFix message
            gnss_msg = NavSatFix()
            gnss_msg.header.stamp = rospy.Time.now()
            gnss_msg.latitude = msg.lat / 1e7
            gnss_msg.longitude = msg.lon / 1e7
            gnss_msg.altitude = msg.alt / 1e3

            # Publish the GNSS data
            pub.publish(gnss_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_gnss()
    except rospy.ROSInterruptException:
        pass