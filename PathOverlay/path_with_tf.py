import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from PIL import Image
from pathlib import Path
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R

# --------------------------
# STEP 1: Extract TF messages and convert to transformation matrices
# --------------------------

def quaternion_matrix(quaternion):
    """
    Convert a quaternion [x, y, z, w] into a 4x4 homogeneous transformation matrix.
    """
    rot = R.from_quat(quaternion)
    rot_matrix = rot.as_matrix()  # 3x3 rotation matrix
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    return T

def get_transform_matrix(transform):
    """
    Given a TransformStamped message, return its 4x4 homogeneous transformation matrix.
    """
    # Translation
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z

    # Rotation quaternion in order [x, y, z, w]
    qx = transform.transform.rotation.x
    qy = transform.transform.rotation.y
    qz = transform.transform.rotation.z
    qw = transform.transform.rotation.w

    T = quaternion_matrix([qx, qy, qz, qw])
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T

def extract_tf_matrices(bag_path, target_parent="map", target_child="odom"):
    """
    Extract TF messages from the rosbag that provide the transform from target_child frame
    to target_parent frame. Returns a list of tuples: (timestamp, T) where T is the 4x4 matrix.
    """
    tf_list = []
    with AnyReader([bag_path]) as reader:
        # Look for both /tf and /tf_static topics
        connections = [conn for conn in reader.connections if conn.topic in ['/tf', '/tf_static']]
        for connection, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            # Each msg typically contains a list of transforms.
            for transform in msg.transforms:
                # We only care about the transform from target_parent to target_child.
                # In TF messages, header.frame_id is the parent frame and child_frame_id is the child.
                if (transform.header.frame_id == target_parent and 
                    transform.child_frame_id == target_child):
                    T = get_transform_matrix(transform)
                    tf_list.append((timestamp, T))
    if not tf_list:
        raise RuntimeError(f"No TF messages found for transform from '{target_parent}' to '{target_child}'")
    print(f"Extracted {len(tf_list)} TF messages for transform {target_parent} -> {target_child}")
    return tf_list

def get_transform_for_timestamp(timestamp, tf_list):
    """
    Given a timestamp and a list of (timestamp, T) tuples, return the transformation matrix
    T whose timestamp is closest to the given timestamp.
    """
    best_T = None
    best_dt = float("inf")
    for ts, T in tf_list:
        dt = abs(ts - timestamp)
        if dt < best_dt:
            best_dt = dt
            best_T = T
    return best_T

# --------------------------
# STEP 2: Load odometry data and transform it into the map frame
# --------------------------

# Path to your rosbag containing TF messages
bag_path = Path(r'C:\Users\Danii\Documents\rosbag2_2025_03_07-11_33_28')  # adjust as needed

# Extract the TF matrices from the rosbag (from map -> odom transform)
tf_list = extract_tf_matrices(bag_path, target_parent="map", target_child="odom")

# Load the odometry CSV (assumed to be in the odom frame)
csv_filename = 'data20/odometry_data.csv'
df = pd.read_csv(csv_filename)
if 'pos_x' not in df.columns or 'pos_y' not in df.columns or 'timestamp' not in df.columns:
    raise ValueError("CSV file must contain 'timestamp', 'pos_x', and 'pos_y' columns.")

# For each odometry point, get its timestamp and coordinate (assume z=0)
odom_timestamps = df['timestamp'].values
odom_points = df[['pos_x', 'pos_y']].values

# Transform the odometry points from odom to map frame.
transformed_points = []
for t, pt in zip(odom_timestamps, odom_points):
    T = get_transform_for_timestamp(t, tf_list)
    if T is None:
        # If no transform found, skip this point.
        continue
    # Convert 2D point to homogeneous coordinate (z = 0, homogeneous coordinate = 1)
    p_hom = np.array([pt[0], pt[1], 0, 1])
    p_transformed = T @ p_hom
    transformed_points.append(p_transformed[:2])
transformed_points = np.array(transformed_points)

# --------------------------
# STEP 3: Load the map image and overlay the transformed path
# --------------------------

# Load the map image (exported as .pgm)
map_filename = 'data20/map_data.pgm'
map_img = Image.open(map_filename)
map_data = np.array(map_img)

# Use map metadata from the TF? In many cases, the OccupancyGrid message carries resolution and origin.
# For now, we assume the map metadata is known or extracted separately.
# Here we use default values (adjust as needed or extract them from a /map message):
default_map_resolution = 0.05   # meters per pixel
default_origin_x = -8.0         # world x coordinate corresponding to the left side of the image
default_origin_y = -5.0         # world y coordinate corresponding to the bottom of the image

# ----- CONFIGURATION -----
# Set this flag to True to automatically extract metadata from rosbag.
use_auto_metadata = True

# ----- AUTOMATIC EXTRACTION OF MAP METADATA -----
if use_auto_metadata:
    try:
        map_metadata = {}

        with AnyReader([bag_path]) as reader:
            # Look for messages on the '/map' topic
            connections = [conn for conn in reader.connections if conn.topic == '/map']
            for connection, timestamp, rawdata in reader.messages(connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                # Extract the resolution and origin from the OccupancyGrid message
                map_metadata['resolution'] = msg.info.resolution
                map_metadata['width'] = msg.info.width
                map_metadata['height'] = msg.info.height
                map_metadata['origin_x'] = msg.info.origin.position.x
                map_metadata['origin_y'] = msg.info.origin.position.y
                # Optionally, you can extract orientation if needed
                map_metadata['origin_orientation'] = (
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w,
                )
                print("Extracted map metadata:")
                print(map_metadata)
                break  # Use the first map message found

            map_resolution = map_metadata.get('resolution')
            offset = 2.15
            origin_x = map_metadata.get('origin_x') * offset
            origin_y = map_metadata.get('origin_y') * offset
    except Exception as e:
        print("Automatic extraction of map metadata failed:", e)
        print("Falling back to default parameters.")
        map_resolution = default_map_resolution
        origin_x = default_origin_x
        origin_y = default_origin_y
else:
    map_resolution = default_map_resolution
    origin_x = default_origin_x
    origin_y = default_origin_y

# Get dimensions from the image
map_height, map_width = map_data.shape
extent = [
    origin_x,
    origin_x + map_width * map_resolution,
    origin_y,
    origin_y + map_height * map_resolution
]

# --------------------------
# Plot the map with the transformed path overlaid
# --------------------------
plt.figure(figsize=(10, 8))
plt.imshow(map_data, cmap='gray', extent=extent, origin='lower')

# Overlay the transformed path
plt.plot(transformed_points[:, 0], transformed_points[:, 1], marker='o', linestyle='-',
         color='red', linewidth=2, markersize=3, alpha=0.5, label='Transformed Odometry Path')

plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Robot Path Transformed from Odom to Map Frame")
plt.legend()
plt.grid(True)
plt.xlim(extent[0], extent[1])
plt.ylim(extent[2], extent[3])

plt.show()

# Optionally, save the overlay to a file.
# plt.savefig('path_with_tf_on_map.png', dpi=300)