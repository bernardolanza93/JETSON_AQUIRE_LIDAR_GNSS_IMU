import rosbag
from sensor_msgs.msg import Image, Imu, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import sys
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from SLAM_V1_decoder_to_map import *
import  math

import sys
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import rosbag
from sensor_msgs.point_cloud2 import read_points
import pyslam   #DA USARE PER VISUAL SALM CON RGB IMAGES
from sensor_msgs.msg import PointCloud2, Imu
import rosbag
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import re
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.spatial.transform import Rotation as R




# Print the Python version being used
print("Python version:", sys.version)


def calculate_fps_and_count(bag_file, rgb_topic, depth_topic):
    bridge = CvBridge()
    rgb_count = 0
    depth_count = 0
    rgb_first_timestamp = None
    rgb_last_timestamp = None
    depth_first_timestamp = None
    depth_last_timestamp = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[rgb_topic, depth_topic]):
            try:
                if topic == rgb_topic:
                    rgb_count += 1
                    if rgb_first_timestamp is None:
                        rgb_first_timestamp = t.to_sec()
                    rgb_last_timestamp = t.to_sec()
                elif topic == depth_topic:
                    depth_count += 1
                    if depth_first_timestamp is None:
                        depth_first_timestamp = t.to_sec()
                    depth_last_timestamp = t.to_sec()
            except CvBridgeError as e:
                print(e)
                continue

    # Calcolo degli FPS
    if rgb_count > 1:
        rgb_duration = rgb_last_timestamp - rgb_first_timestamp
        rgb_fps = rgb_count / rgb_duration
    else:
        rgb_fps = 0

    if depth_count > 1:
        depth_duration = depth_last_timestamp - depth_first_timestamp
        depth_fps = depth_count / depth_duration
    else:
        depth_fps = 0

    print(f"Numero totale di immagini RGB: {rgb_count}")
    print(f"FPS delle immagini RGB: {rgb_fps:.2f}")
    print(f"Numero totale di immagini di profondità: {depth_count}")
    print(f"FPS delle immagini di profondità: {depth_fps:.2f}")

def list_topics(bag_file):
    with rosbag.Bag(bag_file, 'r') as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics.keys()
        print("Topics presenti nel file bag:")
        for topic in topics:
            print(topic)

# Function to convert ROS PointCloud2 message to Open3D point cloud
def convert_point_cloud2_to_open3d(msg):
    # Extract points from the PointCloud2 message
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    # Convert points to a NumPy array
    np_points = np.array(points, dtype=np.float32)

    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)

    return pcd



def alpha_version_main_list_topic_read_bag_generic_file():
    # Define directories and file extensions
    root_directory = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU"
    bag_directory = os.path.join(root_directory, "data")
    extension = ".bag"

    # Traverse the directory to find all .bag files
    for root, dirs, files in os.walk(bag_directory):
        for filename in files:
            if filename.endswith(extension):
                print(filename)

                # Placeholder for any further processing

    # Path to the specific .bag file

    # Path to the specific .bag file
    #bag_file = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/data/zed_20240517_150359.bag"
    bag_file = "/media/mmt-ben/EXTERNAL_US/Assajos/Prova dinamica ETSEA 24-05-24/d455/d455_20240524_121046.orig.bag"

    # Topic per D455 RGB e depth
    rgb_topic = "/camera/color/image_raw"
    depth_topic = "/camera/depth/image_raw"


    list_topics(bag_file)

# Function to extract and print IMU data from the bag fil

# Function to extract and display images from the bag file


def extract_and_save_gnss_data(bag_file, topic_gnss="/GNSS", output_file="app/localization_data/gnss_data.json"):
    gnss_data = []

    def latlon_to_ecef(lat, lon, alt):
        # Parametri dell'ellissoide WGS-84
        a = 6378137.0  # semi-major axis
        f = 1 / 298.257223563  # flattening
        e2 = 2 * f - f ** 2  # eccentricità al quadrato

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)

        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + alt) * math.sin(lat_rad)

        return x, y, z

    def ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref):
        # Conversione delle coordinate del punto di riferimento in ECEF
        x_ref, y_ref, z_ref = latlon_to_ecef(lat_ref, lon_ref, alt_ref)

        # Differenza tra le coordinate ECEF del punto e del punto di riferimento
        dx = x - x_ref
        dy = y - y_ref
        dz = z - z_ref

        # Conversione della latitudine e longitudine del riferimento in radianti
        lat_ref_rad = math.radians(lat_ref)
        lon_ref_rad = math.radians(lon_ref)

        # Matrice di rotazione da ECEF a ENU
        t_matrix = [
            [-math.sin(lon_ref_rad), math.cos(lon_ref_rad), 0],
            [-math.sin(lat_ref_rad) * math.cos(lon_ref_rad), -math.sin(lat_ref_rad) * math.sin(lon_ref_rad), math.cos(lat_ref_rad)],
            [math.cos(lat_ref_rad) * math.cos(lon_ref_rad), math.cos(lat_ref_rad) * math.sin(lon_ref_rad), math.sin(lat_ref_rad)]
        ]

        # Moltiplicazione matrice-vettore per ottenere ENU
        x_local = t_matrix[0][0] * dx + t_matrix[0][1] * dy + t_matrix[0][2] * dz
        y_local = t_matrix[1][0] * dx + t_matrix[1][1] * dy + t_matrix[1][2] * dz
        z_local = t_matrix[2][0] * dx + t_matrix[2][1] * dy + t_matrix[2][2] * dz

        return x_local, y_local, z_local

    lat_ref, lon_ref, alt_ref = None, None, None  # Punto di riferimento

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_gnss]):
            # Estrai dati dal messaggio GNSS
            match = re.match(r".*GNGGA,(\d{6}\.\d{2}),(\d{2})(\d{2}\.\d+),([NS]),(\d{3})(\d{2}\.\d+),([EW]),.*",
                             msg.data)
            if match:
                time_gnss = match.group(1)
                lat_deg = float(match.group(2)) + float(match.group(3)) / 60.0
                if match.group(4) == 'S':
                    lat_deg = -lat_deg
                lon_deg = float(match.group(5)) + float(match.group(6)) / 60.0
                if match.group(7) == 'W':
                    lon_deg = -lon_deg
                alt = float(re.search(r",(\d+\.\d+),M,", msg.data).group(1))

                x, y, z = latlon_to_ecef(lat_deg, lon_deg, alt)

                # Imposta il punto di riferimento (primo punto)
                if lat_ref is None:
                    lat_ref, lon_ref, alt_ref = lat_deg, lon_deg, alt

                # Converti da ECEF a ENU e rinomina in x, y, z
                x_local, y_local, z_local = ecef_to_enu(x, y, z, lat_ref, lon_ref, alt_ref)

                gnss_entry = {
                    'timestamp': t.to_sec(),
                    'position': {
                        'x': x_local,
                        'y': y_local,
                        'z': z_local
                    }
                }
                gnss_data.append(gnss_entry)

    with open(output_file, 'w') as f:
        json.dump(gnss_data, f, indent=4)


def save_pointclouds_with_poses(pointclouds, poses, output_dir='results'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for i, (pc, pose) in enumerate(zip(pointclouds, poses)):
        transformed_pc = pc.transform(pose)
        filename = os.path.join(output_dir, f'pointcloud_{i:03d}.ply')
        o3d.io.write_point_cloud(filename, transformed_pc)

# Function to extract and display point clouds from the bag file

def load_pointclouds_from_rosbag(bag_file, pc_topic, imu_topic):
    bag = rosbag.Bag(bag_file)
    pointclouds = []
    pc_timestamps = []
    imu_data = []
    imu_timestamps = []
    ii = 0

    for topic, msg, t in bag.read_messages(topics=[pc_topic, imu_topic]):


        ii += 1

        if topic == pc_topic:
            pc = pointcloud2_to_o3d(msg)
            pointclouds.append(pc)
            pc_timestamps.append(t.to_sec())
        elif topic == imu_topic:
            imu_data.append(msg)
            imu_timestamps.append(t.to_sec())

        print("extraction bag:",ii)



    bag.close()
    return pointclouds, pc_timestamps, imu_data, imu_timestamps



def save_imu_data_to_file(imu_data, imu_timestamps, filename="imu_data.json"):
    data = {
        "timestamps": imu_timestamps,
        "linear_acceleration_x": [imu.linear_acceleration.x for imu in imu_data],
        "linear_acceleration_y": [imu.linear_acceleration.y for imu in imu_data],
        "linear_acceleration_z": [imu.linear_acceleration.z for imu in imu_data],
        "angular_velocity_x": [imu.angular_velocity.x for imu in imu_data],
        "angular_velocity_y": [imu.angular_velocity.y for imu in imu_data],
        "angular_velocity_z": [imu.angular_velocity.z for imu in imu_data],
    }
    with open(filename, 'w') as f:
        json.dump(data, f)

def extract_dump_imu_orientation(bag_file, topic_gnss="/imu/data", output_file="imu_partial_xsens_data.json"):
    imu_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_gnss]):
            imu_entry = {
                'header': {
                    'seq': msg.header.seq,
                    'stamp': {
                        'secs': msg.header.stamp.secs,
                        'nsecs': msg.header.stamp.nsecs
                    },
                    'frame_id': msg.header.frame_id
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'orientation_covariance': list(msg.orientation_covariance),
                'timestamp': {
                    'secs': t.secs,
                    'nsecs': t.nsecs
                }
            }
            imu_data.append(imu_entry)

    with open(output_file, 'w') as f:
        json.dump(imu_data, f, indent=4)


def save_imu_kinecct_to_file(bag_file, topic_imu = "/imu/data", output_json="app/localization_data/imu_data.json"):
    # Dictionary to hold all the accelerometer data
    imu_data = {}

    # Open the bag file and read messages
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_imu]):
            # Extract the timestamp
            timestamp = f"{msg.header.stamp.secs}.{msg.header.stamp.nsecs}"

            # Extract the IMU data
            data = {
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z
                }
            }

            # Add the data to the dictionary with the timestamp as the key
            imu_data[timestamp] = data


    # Save the data to a JSON file
    with open(output_json, 'w') as file:
        json.dump(imu_data, file, indent=4)

if __name__ == "__main__":

    #main()
    #main_zed()
    bag_file = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/imu_kinect_xsens/ka_20240716_113100.bag"
    topic_imu  = "/imu/data"

    list_topics(bag_file)

    extract_and_save_gnss_data(bag_file)
    #save_imu_kinecct_to_file(bag_file)



    #extract_and_save_gnss_data(bag_file)



