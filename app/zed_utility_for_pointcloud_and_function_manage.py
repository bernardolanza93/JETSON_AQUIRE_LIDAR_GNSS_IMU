import rosbag
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
import csv
import sys
import json
import math
import os
import cv2
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
from scipy.integrate import cumtrapz

import numpy as np
from scipy.spatial.transform import Rotation as R




def extract_images_zed(bag_file, topic_rgb, topic_depth, topic_camera_info, save_folder_rgb, save_folder_depth,
                       save_folder_intrinsics):
    # Verifica se le cartelle di salvataggio esistono, altrimenti le crea
    if not os.path.exists(save_folder_rgb):
        os.makedirs(save_folder_rgb)
    if not os.path.exists(save_folder_depth):
        os.makedirs(save_folder_depth)
    if not os.path.exists(save_folder_intrinsics):
        os.makedirs(save_folder_intrinsics)

    bridge = CvBridge()
    camera_info_saved = False

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_rgb, topic_depth, topic_camera_info]):
            timestamp = t.to_sec()
            if topic == topic_rgb:
                # Converti il messaggio Image in un'immagine OpenCV
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # Salva l'immagine RGB con il timestamp come nome file
                rgb_filename = os.path.join(save_folder_rgb, f"{timestamp}.png")
                cv2.imwrite(rgb_filename, cv_image)
            elif topic == topic_depth:
                # Converti il messaggio Image in un'immagine OpenCV
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                # Salva l'immagine di profondità con il timestamp come nome file
                depth_filename = os.path.join(save_folder_depth, f"{timestamp}.png")
                cv2.imwrite(depth_filename, cv_image)
            elif topic == topic_camera_info and not camera_info_saved:
                # Salva le intrinseche della fotocamera solo una volta
                intrinsics_filename = os.path.join(save_folder_intrinsics, "intrinsics_zed.txt")
                with open(intrinsics_filename, 'w') as file:
                    file.write(f"width: {msg.width}\n")
                    file.write(f"height: {msg.height}\n")
                    file.write(f"distortion_model: {msg.distortion_model}\n")
                    file.write(f"D: {msg.D}\n")
                    file.write(f"K: {msg.K}\n")
                    file.write(f"R: {msg.R}\n")
                    file.write(f"P: {msg.P}\n")
                camera_info_saved = True

    print(f"Immagini RGB salvate in {save_folder_rgb}")
    print(f"Immagini di profondità salvate in {save_folder_depth}")
    print(f"Intrinseche della fotocamera salvate in {save_folder_intrinsics}")


def extract_imu_zed(bag_file, topic_imu, save_folder):

    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
    imu_data = []
    imu_timestamps = []
    vv = 0
    with rosbag.Bag(bag_file, 'r') as bag:

        for topic, msg, t in bag.read_messages(topics=[topic_imu]):
            print("extract imu zed:", vv)
            # Estrarre i dati dell'accelerometro dal messaggio IMU
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z

            # Estrarre i dati del giroscopio dal messaggio IMU
            gyro_x = msg.angular_velocity.x
            gyro_y = msg.angular_velocity.y
            gyro_z = msg.angular_velocity.z

            # Estrarre i dati dell'orientazione (quaternion) dal messaggio IMU
            orient_x = msg.orientation.x
            orient_y = msg.orientation.y
            orient_z = msg.orientation.z
            orient_w = msg.orientation.w

            imu_data.append([accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, orient_x, orient_y, orient_z, orient_w])
            imu_timestamps.append(t.to_sec())
            vv += 1

    # Salva i dati in un file CSV
    output_file = os.path.join(save_folder, 'imu_data.csv')
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Scrivi l'intestazione
        writer.writerow(
            ['Timestamp', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Orient_X', 'Orient_Y',
             'Orient_Z', 'Orient_W'])
        # Scrivi i dati
        for i in range(len(imu_timestamps)):
            writer.writerow([imu_timestamps[i]] + imu_data[i])

    print(f"Dati salvati in {output_file}")
    return imu_data, imu_timestamps

def extract_pointclouds_zed(bag_file, topic_pointcloud):
    pointclouds = []
    pointcloud_timestamps = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_pointcloud]):
            pointclouds.append(msg)
            pointcloud_timestamps.append(t.to_sec())
    return pointclouds, pointcloud_timestamps

def compute_transformation_matrices_zed(imu_data, imu_timestamps):
    transformation_matrices = []
    integrated_angular_velocity = np.array([0.0, 0.0, 0.0])  # Initial orientation (Euler angles)
    last_time = imu_timestamps[0]

    for i, imu_msg in enumerate(imu_data):
        current_time = imu_timestamps[i]
        dt = current_time - last_time
        last_time = current_time

        # Extract angular velocity
        angular_velocity = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])

        # Integrate angular velocity to get Euler angles
        integrated_angular_velocity += angular_velocity * dt
        rotation_matrix = R.from_euler('xyz', integrated_angular_velocity).as_matrix()

        transformation_matrices.append(rotation_matrix)

    return transformation_matrices

def transform_pointclouds(pointclouds, transformation_matrices, pointcloud_timestamps, imu_timestamps):
    transformed_pointclouds = []
    for i, pc in enumerate(pointclouds):
        idx = np.searchsorted(imu_timestamps, pointcloud_timestamps[i])
        if idx < len(transformation_matrices):
            rotation_matrix = transformation_matrices[idx]
            pc_array = np.asarray(list(pc2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True)))
            transformed_points = np.dot(rotation_matrix, pc_array.T).T
            transformed_pointclouds.append(transformed_points)
    return transformed_pointclouds

def save_transformed_pointclouds(transformed_pointclouds, output_file="pointcloud_zed.ply"):
    combined_pc = o3d.geometry.PointCloud()
    for points in transformed_pointclouds:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        combined_pc += pc
    o3d.io.write_point_cloud(output_file, combined_pc)

def visualize_pointcloud(filename="pointcloud_zed.ply"):
    pc = o3d.io.read_point_cloud(filename)
    o3d.visualization.draw_geometries([pc])

def list_topics(bag_file):
    with rosbag.Bag(bag_file, 'r') as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics.keys()
        print("Topics presenti nel file bag:")
        for topic in topics:
            print(topic)


def extract_and_save_pointclouds_zed(bag_file, topic_pointcloud, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_pointcloud]):
            pc = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if pc:
                np_pc = np.array(pc, dtype=np.float32)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np_pc[:, :3])
                timestamp = t.to_sec()
                filename = os.path.join(output_folder, f"{timestamp}.ply")
                if not os.path.exists(filename):
                    o3d.io.write_point_cloud(filename, pcd)
                    print(f"Saved {filename}")
                else:
                    print(f"File {filename} already exists. Skipping save.")

def save_pointclouds(pointclouds, timestamps, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    for pcd, ts in zip(pointclouds, timestamps):
        filename = os.path.join(output_folder, f"{ts}.ply")
        if not os.path.exists(filename):
            o3d.io.write_point_cloud(filename, pcd)
            print(f"Saved {filename}")
        else:
            print(f"File {filename} already exists. Skipping save.")

def main_zed():
    #topics:
    # / GNSS
    # / imu / data
    # / zedxm / zed_node / depth / camera_info
    # / zedxm / zed_node / depth / depth_registered
    # / zedxm / zed_node / imu / data
    # / zedxm / zed_node / imu / data_raw
    # / zedxm / zed_node / left_raw / image_raw_color
    # / zedxm / zed_node / odom
    # / zedxm / zed_node / point_cloud / cloud_registered
    # / zedxm / zed_node / pose
    # / zedxm / zed_node / pose / status
    # / zedxm / zed_node / pose_with_covariance

    bag_file = "/home/mmt-ben/Downloads/zed_20240531_121333.bag"
    topic_zed_pointcloud = "/zedxm/zed_node/point_cloud/cloud_registered"
    output_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_zed_reg"  # Sostituisci con il percorso della cartella di output
    save_folder_imu_zed = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_imu_data"
    topic_zed_imu = "/zedxm/zed_node/imu/data"
    topic_rgb = '/zedxm/zed_node/left_raw/image_raw_color'
    topic_depth = '/zedxm/zed_node/depth/depth_registered'
    topic_camera_info = '/zedxm/zed_node/depth/camera_info'
    save_folder_rgb = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_img/RGB'
    save_folder_depth = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_img/Depth'
    save_folder_intrinsics = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_img/intrinsics'
    topic_odom = '/zedxm/zed_node/odom'
    save_folder_odom = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/odometry_zed'


    extract_images_zed(bag_file, topic_rgb, topic_depth, topic_camera_info, save_folder_rgb, save_folder_depth, save_folder_intrinsics)

    #list_topics(bag_file)



    #
    # # Step 1: Extract IMU data from bag file
    #imu_data, imu_timestamps = extract_imu_zed(bag_file, topic_zed_imu, save_folder_imu_zed)



    # Step 1: Extract point clouds from bag file
    #extract_and_save_pointclouds_zed(bag_file, topic_zed_pointcloud, output_folder)




main_zed()