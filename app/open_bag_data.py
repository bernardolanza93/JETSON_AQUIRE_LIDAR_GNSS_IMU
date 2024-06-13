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
from zed_utility_for_pointcloud_and_function_manage import *
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


def stream_images_D455(bag_file, rgb_topic = "/camera/color/image_raw", depth_topic = "/camera/depth/image_raw"):
    bridge = CvBridge()
    depth_msg_count = 0  # Contatore per i messaggi di profondità

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[rgb_topic, depth_topic]):
            try:
                if topic == rgb_topic:
                    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    cv2.imshow("RGB Image", rgb_image)
                elif topic == depth_topic:
                    depth_msg_count += 1
                    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                    depth_image_normalized = cv2.convertScaleAbs(depth_image_normalized)
                    cv2.imshow("Depth Image", depth_image_normalized)

                # Aspetta 1 ms per la chiave, per terminare premere 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except CvBridgeError as e:
                print(e)
                continue

    print(f"Numero totale di messaggi di profondità letti: {depth_msg_count}")
    cv2.destroyAllWindows()


def extract_images_zed(bag_file):
    # Initialize CvBridge
    bridge = CvBridge()

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/zedxm/zed_node/left_raw/image_raw_color']):
            print(f"Reading message from topic {topic} at time {t.to_sec()}")

            try:
                # Convert ROS message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # Display the image using OpenCV
                cv2.imshow("Image", cv_image)
                cv2.waitKey(0)  # Wait for a key press

                # Debug print
                print(f"Image at time {t.to_sec()} displayed.")

            except CvBridgeError as e:
                print(f"Error converting image: {e}")

    # Close all OpenCV windows when done
    cv2.destroyAllWindows()
def save_pointclouds_with_poses(pointclouds, poses, output_dir='results'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    for i, (pc, pose) in enumerate(zip(pointclouds, poses)):
        transformed_pc = pc.transform(pose)
        filename = os.path.join(output_dir, f'pointcloud_{i:03d}.ply')
        o3d.io.write_point_cloud(filename, transformed_pc)

# Function to extract and display point clouds from the bag file


if __name__ == "__main__":

    #main()
    main_zed()


