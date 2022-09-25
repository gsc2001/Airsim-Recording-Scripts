from __future__ import division
import os
import sys
import argparse
import threading

import rosbag
import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from tqdm import tqdm


class ParseKwargs(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, dict())
        for value in values:
            key, value = value.split('=')
            getattr(namespace, self.dest)[key] = value


parser = argparse.ArgumentParser(
    description='Converts original airsim files (capture/ imu_data.txt) into bagfile format')
parser.add_argument('--dataset-config', required=True, action=ParseKwargs, nargs="*")
parser.add_argument('--percentage', type=int, default=100, help='How much of the dataset to compress')
parser.add_argument('--output', required=True, help='Name of the output file')
args = parser.parse_args()
output_name = args.output

lock = threading.Lock()

def imu(dataset_name, dataset_path, bag, offset):
    global lock
    imu_data_file = open(os.path.join(dataset_path, 'imu_data.txt'), 'r')
    imu_data_file = list(imu_data_file)
    print('IMU')
    for line in tqdm(range(1, int(args.percentage / 100 * len(imu_data_file)))):
        line = imu_data_file[line]
        l = line.rstrip().split("\t")
        l = list(map(eval, l))

        timestamp = rospy.Time.from_sec((l[0] + offset) / 1e9)
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
        lock.acquire()
        bag.write(f"/{dataset_name}/imu", imu_msg, timestamp)
        lock.release()



def gt_data(dataset_name, dataset_path, bag, offset):
    global lock
    gt_data_file = open(os.path.join(dataset_path, 'gt_data.txt'), 'r')
    gt_data_file = list(gt_data_file)
    print('GT DATA')
    for line in tqdm(range(1, int(args.percentage / 100 * len(gt_data_file)))):
        line = gt_data_file[line]
        l = line.rstrip().split("\t")
        l = list(map(eval, l))

        timestamp = rospy.Time.from_sec((l[0] + offset) / 1e9)
        pose = PoseStamped()

        pose.header.stamp = timestamp
        pose.header.frame_id = "world"
        pose.pose.position.x = l[1]
        pose.pose.position.y = l[2]
        pose.pose.position.z = l[3]

        pose.pose.orientation.w = l[4]
        pose.pose.orientation.w = l[4]
        pose.pose.orientation.x = l[5]
        pose.pose.orientation.y = l[6]
        pose.pose.orientation.z = l[7]

        lock.acquire()
        bag.write(f"/{dataset_name}/gt_pose", pose, timestamp)
        lock.release()

def images(dataset_name, dataset_path, bag, offset):
    global lock
    img_data_file = open(os.path.join(dataset_path, "captures/timestamps.txt"), 'r')
    img_data_file = list(img_data_file)
    cb = CvBridge()
    print('Images')
    for i in tqdm(range(int(args.percentage / 100 * len(img_data_file)))):
        line = img_data_file[i]
        l = line.rstrip().split("\t")
        timestamp = rospy.Time.from_sec((int(l[0]) + offset) / 1e9)
        img_path = os.path.join(dataset_path, ('captures/' + str(i) + '_scene.png'))
        img_rgb = cv2.imread(img_path)
        img_path = os.path.join(dataset_path, ('captures/' + str(i) + '_depth.png'))
        img_depth = cv2.imread(img_path, cv2.IMREAD_ANYDEPTH)
        img_rgb_msg = cb.cv2_to_imgmsg(img_rgb, encoding='bgr8')
        img_rgb_msg.header.stamp = timestamp
        img_rgb_msg.header.frame_id = l[1]
        img_depth_msg = cb.cv2_to_imgmsg(img_depth, encoding='mono16')
        img_depth_msg.header.stamp = timestamp
        img_depth_msg.header.frame_id = l[1]
        lock.acquire()
        bag.write(f"/{dataset_name}/image", img_rgb_msg, timestamp)
        bag.write(f"/{dataset_name}/depth", img_depth_msg, timestamp)
        lock.release()



threads = []
with rosbag.Bag(os.path.join(output_name), 'w') as bag:
    for dataset_name, dataset_path in args.dataset_config.items():

        if dataset_name == 'd1':
            offset = 1652107109004886784 - 1652105281589408770
        # Write IMU messages
        print('Dataset', dataset_name)
        threads.append(threading.Thread(target=imu, args=(dataset_name, dataset_path, bag, offset, )))
        threads.append(threading.Thread(target=gt_data, args=(dataset_name, dataset_path, bag, offset, )))
        threads.append(threading.Thread(target=images, args=(dataset_name, dataset_path, bag, offset, )))

    [t.start() for t in threads]
    [t.join() for t in threads]

# d1 is 25 behind d2 in x
