#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class ImuToRpy():
    """ Class to get the orientation messages of Imu and spit them out
    again in roll pitch yaw in another topic for ease of debugging.
    """
    def __init__(self):
        self.rpy_pub = rospy.Publisher('myo_imu_rpy', Float64MultiArray, queue_size=10)
        self.imu_sub = rospy.Subscriber('myo_imu', Imu, self.imu_cb, queue_size=10)

    def imu_cb(self, data):
        """:type data: Imu"""
        roll, pitch, yaw = euler_from_quaternion([data.orientation.x,
                                                 data.orientation.y,
                                                 data.orientation.z,
                                                 data.orientation.w])
        rpy_msg = Float64MultiArray()
        rpy_msg.data.append(roll)
        rpy_msg.data.append(pitch)
        rpy_msg.data.append(yaw)
        self.rpy_pub.publish(rpy_msg)


if __name__ == '__main__':
    rospy.init_node('myo_imu_rpy_node', anonymous=True)
    itrpy = ImuToRpy()
    rospy.spin()
