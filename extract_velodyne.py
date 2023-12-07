import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def extract_velodyne_points(input_bag_file, output_bin_file):
    with rosbag.Bag(input_bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == '/velodyne_points':
                point_cloud = pc2.read_points(msg)
                point_cloud_array = np.array(list(point_cloud))
                # Save the point cloud as a binary file
                point_cloud_array.tofile(output_bin_file)

if __name__ == "__main__":
    input_bag_file = '/home/ubuntu/DATA/Test_6.bag'
    output_bin_file = "velodyne_points.bin"
    extract_velodyne_points(input_bag_file, output_bin_file)
