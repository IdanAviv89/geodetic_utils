#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from GpsSimulator import GpsSimulator
from tf import transformations

class GpsSimNode:

    def __init__(self):
        rospy.init_node('gps_sim_node', anonymous=True)

        self.odom_pub = rospy.Publisher('gps_out', Odometry, queue_size=10)
        self.mag_pub = rospy.Publisher('mag_out', MagneticField, queue_size=10)

        self.gps_sim = GpsSimulator()
        rospy.Subscriber("/HeadHelmety/vrpn_client/estimated_odometry", Odometry, self.odom_callback)

    def odom_callback(self, data):
        # extract pose
        input_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        input_pose = transformations.quaternion_matrix(np.array([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]))

        # assmeble 4x4 matrix
        input_pose[0:3,3] = input_pos
        input_stamp = data.header.stamp

        enu_pos, enu_cov, mag_data = self.gps_sim.simulate(input_pose)

        # assemble messages.
        mag_msg = MagneticField()
        mag_msg.header.stamp = input_stamp
        mag_msg.magnetic_field.x = mag_data[0]
        mag_msg.magnetic_field.y = mag_data[1]
        mag_msg.magnetic_field.z = mag_data[2]

        print(np.arctan2(mag_data[1], mag_data[0]) * (180 / 3.1415))
        self.mag_pub.publish(mag_msg)

        if enu_pos is None or enu_cov is None:
            # no gps data :(
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = input_stamp
        odom_msg.pose.pose.position.x = enu_pos[0]
        odom_msg.pose.pose.position.y = enu_pos[1]
        odom_msg.pose.pose.position.z = enu_pos[2]
        odom_msg.pose.covariance[0] = enu_cov[0, 0]
        odom_msg.pose.covariance[1] = enu_cov[0, 1]
        odom_msg.pose.covariance[2] = enu_cov[0, 2]
        odom_msg.pose.covariance[6] = enu_cov[1, 0]
        odom_msg.pose.covariance[7] = enu_cov[1, 1]
        odom_msg.pose.covariance[8] = enu_cov[1, 2]
        odom_msg.pose.covariance[12] = enu_cov[2, 0]
        odom_msg.pose.covariance[13] = enu_cov[2, 1]
        odom_msg.pose.covariance[14] = enu_cov[2, 2]
        self.odom_pub.publish(odom_msg)


if __name__ == '__main__':
    node = GpsSimNode()
    rospy.spin()
