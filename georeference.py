import pandas as pd
import os
import numpy as np
import open3d as o3d
from tqdm import tqdm
import matplotlib.cm as cm
import laspy
import pyproj

# =========================
# Step 1: Data Loading
# =========================

# Load IMU data
imu_data = pd.read_csv('/home/ubuntu/DATA/IMU_DATA.csv')

# Load GNSS data
gnss_data = pd.read_csv('/home/ubuntu/DATA/FIX-Test-6.pos', delim_whitespace=True, skiprows=10, header=None)
gnss_data['timestamp'] = (pd.to_datetime(gnss_data[0] + ' ' + gnss_data[1]) - pd.Timestamp("1970-01-01")) // pd.Timedelta('1s')

# Load PCD data
pcd_files = sorted([os.path.join("/home/ubuntu/DATA/PATH_3", f) for f in os.listdir("/home/ubuntu/DATA/PATH_3") if f.endswith('.pcd')])

print("Loading data completed.")

# =========================
# Step 2: Synchronization
# =========================

def get_closest_timestamp_idx(target_timestamp, timestamp_series):
    closest_idx = (timestamp_series - target_timestamp).abs().idxmin()
    closest_timestamp = timestamp_series.iloc[closest_idx]
    
    # Only consider timestamps within a 1-second difference
    if abs(closest_timestamp - target_timestamp) <= 1:
        return closest_idx
    return None

def normalize_quaternion(q):
    mag = np.linalg.norm(q)
    return q / mag

# Function to convert WGS84 to UTM 48N
def convert_to_utm(lat, lon, height):
    utm_proj = pyproj.Proj(proj='utm', zone=48, ellps='WGS84', south=False)
    x, y = utm_proj(lon, lat)
    return np.array([x, y, height])

# Flags for initial reference
has_set_initial_reference = False

synchronized_data = []

# Iterate through GNSS data for synchronization
for idx, row in tqdm(gnss_data.iterrows(), desc="Synchronizing Data"):
    gnss_timestamp = row['timestamp']

    pcd_file_timestamps = [float(os.path.basename(f).replace('.pcd', '')) for f in pcd_files]
    pcd_idx = get_closest_timestamp_idx(gnss_timestamp, pd.Series(pcd_file_timestamps))
    imu_idx = get_closest_timestamp_idx(gnss_timestamp, imu_data['timestamp'])

    if pcd_idx is None or imu_idx is None:
        continue  # skip if no corresponding data found within 1-second tolerance

    pcd_file = pcd_files[pcd_idx]
    imu_orientation = imu_data.iloc[imu_idx][['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']].values
    normalized_quaternion = normalize_quaternion(imu_orientation)

    gnss_position_wgs84 = row[2:5].values.astype(float)
    gnss_position = convert_to_utm(*gnss_position_wgs84)
    gnss_position[2] = 22.0

    synchronized_data.append({
        'pcd_file': pcd_file,
        'imu_orientation': normalized_quaternion,
        'gnss_position': gnss_position
    })

print("Synchronization completed.")

# =========================
# Step 3: Direct Geo-referencing & Point Cloud Generation
# =========================

def quaternion_to_rotation_matrix(x, y, z, w):
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z -y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

def lidar_to_imu_transformation(lidar_coords):
    R = np.array([
        [0, -1, 0],
        [0, 0, 1],
        [-1, 0, 0]
    ])
    return np.dot(lidar_coords, R.T)

global_cloud_points = []

for data in tqdm(synchronized_data, desc="Direct Geo-referencing"):
    # Load and transform Lidar points to IMU coordinate system
    cloud_o3d = o3d.io.read_point_cloud(data['pcd_file'])
    cloud_np = np.asarray(cloud_o3d.points)
    cloud_in_imu = lidar_to_imu_transformation(cloud_np)
    
    # Compute Rotation Matrix from IMU orientation
    x, y, z, w = data['imu_orientation']
    rotation_matrix = quaternion_to_rotation_matrix(x, y, z, w)
    
    # Transform points to global coordinate system
    global_coordinates = np.dot(cloud_in_imu, rotation_matrix.T) + data['gnss_position']
    
    # Aggregate points
    global_cloud_points.append(global_coordinates)

# Concatenate all points to form the global point cloud
global_cloud_np = np.vstack(global_cloud_points)
global_cloud_o3d = o3d.geometry.PointCloud()
global_cloud_o3d.points = o3d.utility.Vector3dVector(global_cloud_np)

print("Direct Geo-referencing completed.")

# =========================
# Step 4: Colorizing and Saving as LAS
# =========================

def colorize_point_cloud_by_elevation(point_cloud_np):
    z_values = point_cloud_np[:, 2].astype(float)  # Assuming Z is the elevation
    min_z = np.min(z_values)
    max_z = np.max(z_values)

    # Create a colormap
    colors = cm.jet((z_values - min_z) / (max_z - min_z))

    # Convert RGB values from [0, 1] to [0, 255]
    colors = (colors[:, :3] * 255).astype(np.uint8)
    
    return colors

# Colorize the point cloud by elevation
colors = colorize_point_cloud_by_elevation(global_cloud_np)

output_path = '/home/ubuntu/DATA/STREET.las'

# Create a new LAS object using laspy
las = laspy.create(point_format=2)

# Add header information
las.header.offsets = [0.0, 0.0, 0.0]
las.header.scales = [0.01, 0.01, 0.01]

las.X = (global_cloud_np[:, 0] / las.header.scales[0]).astype(np.float64)
las.Y = (global_cloud_np[:, 1] / las.header.scales[1]).astype(np.float64)
las.Z = (global_cloud_np[:, 2] / las.header.scales[2]).astype(np.float64)

las.red = colors[:, 0]
las.green = colors[:, 1]
las.blue = colors[:, 2]

# Write the LAS file
las.write(output_path)

print(f"Color Coding completed using laspy. LAS file saved to {output_path}.")
