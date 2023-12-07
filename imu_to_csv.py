#!/usr/bin/env python

import rosbag
import csv
import sys

if len(sys.argv) < 3:
    print("Usage: imu_to_csv.py <input bag file> <output csv file>")
    sys.exit(0)

bag_file = sys.argv[1]
csv_file = sys.argv[2]

with rosbag.Bag(bag_file, 'r') as bag, open(csv_file, 'w') as csvfile:
    writer = csv.writer(csvfile)
    # Write header
    writer.writerow(["timestamp", "orientation.x", "orientation.y", "orientation.z", "orientation.w",
                     "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                     "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"])
    
    for topic, msg, t in bag.read_messages(topics=['/imu/data']):
        writer.writerow([t.to_sec(), 
                         msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                         msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

print("Data exported to {}".format(csv_file))
