#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from math import radians, pi

def gen_msg(x, y, z, roll, pitch, yaw):
    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = "world"

    # Create a quaternion from roll, pitch, and yaw angles
    quaternion = tf.transformations.quaternion_from_euler(
        (roll), (pitch), (yaw))

    initial_pose_msg.pose.pose.position.x = x
    initial_pose_msg.pose.pose.position.y = y
    initial_pose_msg.pose.pose.position.z = z
    initial_pose_msg.pose.pose.orientation.x = quaternion[0]
    initial_pose_msg.pose.pose.orientation.y = quaternion[1]
    initial_pose_msg.pose.pose.orientation.z = quaternion[2]
    initial_pose_msg.pose.pose.orientation.w = quaternion[3]

    # Set covariance values
    initial_pose_msg.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
    initial_pose_msg.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
    initial_pose_msg.pose.covariance[6 * 3 + 3] = pi / 12.0 * pi / 12.0

    return initial_pose_msg

def publish_initial_pose(x, y, z, roll, pitch, yaw):
    rospy.init_node('initial_pose_publisher', anonymous=True)
    initial_pose_pub = rospy.Publisher(
        'initialpose', PoseWithCovarianceStamped, queue_size=1)
    rate = rospy.Rate(2)  # 2 Hz

    initial_pose_msg = gen_msg(x, y, z, roll, pitch, yaw)

    cnt = 0
    while not rospy.is_shutdown():
        initial_pose_msg = gen_msg(x, y, z, roll, pitch, yaw)
        initial_pose_pub.publish(initial_pose_msg)
        
        print("[pub_initialpose] pub msg:",str(initial_pose_msg))
        cnt = cnt + 1
        if cnt >= 2:
            break
        rate.sleep()
    return str(initial_pose_msg)


if __name__ == '__main__':
    if len(sys.argv) != 7:
        print("Usage: pub_initialpose.py x y z roll pitch yaw. In radians.") 
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    roll = float(sys.argv[4])
    pitch = float(sys.argv[5])
    yaw = float(sys.argv[6])


    print("[pub_initialpose] called to set position: {} {} {} {} {} {}".format(x, y, z, roll, pitch, yaw))

    try:
        str_result = publish_initial_pose(x, y, z, roll, pitch, yaw)
        # print("[pub_initialpose] pub msg:",str_result)
    except rospy.ROSInterruptException:
        pass
