from __future__ import division
import os
import sys
import argparse

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description='Converts original airsim files (capture/ imu_data.txt) into bagfile format')
parser.add_argument('dataset_path', help='Path to original airsim files')
args = parser.parse_args()
dataset_path = args.dataset_path

with rosbag.Bag(os.path.join(dataset_path,'masked_sequence.bag'), 'w') as bag:
    # Write IMU messages
    imu_data_file = open(os.path.join(dataset_path,'imu_data.txt'), 'r')

    # Ignore header
    next(imu_data_file)
    for line in imu_data_file:
        l = line.rstrip().split("\t")
        l = list(map(eval, l))

        timestamp = rospy.Time.from_sec((l[0])/1e9)
        imu_msg = Imu()

        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = "imu"
        imu_msg.angular_velocity.x = l[1]
        imu_msg.angular_velocity.y = l[2]
        imu_msg.angular_velocity.z = l[3]

        imu_msg.linear_acceleration.x = l[4]
        imu_msg.linear_acceleration.y = l[5]
        imu_msg.linear_acceleration.z = l[6]

        imu_msg.orientation.w = l[7]
        imu_msg.orientation.x = l[8]
        imu_msg.orientation.y = l[9]
        imu_msg.orientation.z = l[10]

        bag.write("/imu", imu_msg, timestamp)

    # Write image and mask messages
    img_data_file = open(os.path.join(dataset_path, "captures/timestamps.txt"), 'r')
    cb = CvBridge()
    for line in img_data_file:
        l = line.rstrip().split("\t")
        timestamp = rospy.Time.from_sec((eval(l[0]))/1e9)
        img_path = os.path.join(dataset_path, ('captures/' + str(l[1]) + '_rgb.png'))
        img_rgb = cv2.imread(img_path)

        mask_path = os.path.join(dataset_path, ('captures/' + str(l[1]) + '_seg.png'))
        mask = cv2.imread(mask_path,0)
        img_masked = cv2.bitwise_and(img_rgb,img_rgb,mask=mask)
        img_masked = cb.cv2_to_imgmsg(img_masked, encoding='bgr8')
        img_masked.header.stamp = timestamp
        img_masked.header.frame_id = l[1]
        bag.write("/masked_image", img_masked, timestamp)
