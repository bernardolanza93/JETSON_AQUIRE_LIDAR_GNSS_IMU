import os
import ctypes
import pyk4a
from pyk4a import PyK4APlayback, Config, PyK4A
import pandas as pd
from pyk4a import PyK4APlayback, ImageFormat
import open3d as o3d
import numpy as np
import sys
import open3d as o3d
import glob
import cv2
import os
import subprocess
import open3d as o3d
import glob
import open3d as o3d
import numpy as np
import os
from pyk4a import PyK4APlayback, CalibrationType
from pyk4a import PyK4A
from pyk4a import Config, PyK4APlayback





def load_intrinsics():
    # Hardcoded intrinsics based on the provided parameters
    intrinsics_matrix = np.array([[362.6983642578125, 0.0, 471.6519775390625],
                                  [0.0, 362.6983642578125, 303.59527587890625],
                                  [0.0, 0.0, 1.0]], dtype=np.float64)

    return o3d.camera.PinholeCameraIntrinsic(
        width=960, height=600,
        fx=intrinsics_matrix[0, 0], fy=intrinsics_matrix[1, 1], cx=intrinsics_matrix[0, 2], cy=intrinsics_matrix[1, 2]
    )

# Esempio di utilizzo
intrinsics = load_intrinsics()
print(intrinsics)

def load_images(rgb_folder, depth_folder, skip_start=0, skip_end=0):
    rgb_images = []
    depth_images = []
    timestamps = []

    # Ottieni i nomi dei file .png nella cartella rgb_folder
    rgb_files = sorted([f for f in os.listdir(rgb_folder) if f.endswith('.png')])
    depth_files = sorted([f for f in os.listdir(depth_folder) if f.endswith('.png')])

    # Applica lo skip sia all'inizio che alla fine
    rgb_files = rgb_files[skip_start:len(rgb_files) - skip_end]
    depth_files = depth_files[skip_start:len(depth_files) - skip_end]

    for i, rgb_file in enumerate(rgb_files):
        timestamp_rgb = rgb_file.split('.png')[0]
        rgb_path = os.path.join(rgb_folder, rgb_file)

        # Carica l'immagine RGB
        rgb_image = cv2.imread(rgb_path, cv2.IMREAD_COLOR)

        # Usa l'indice per caricare la profondità corrispondente
        if i < len(depth_files):
            depth_file = depth_files[i]
            depth_path = os.path.join(depth_folder, depth_file)

            # Carica l'immagine di profondità
            depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

            # Verifica se le immagini sono state caricate correttamente
            if rgb_image is not None:
                print(f"Loaded RGB image {rgb_path} successfully.")
            else:
                print(f"Failed to load RGB image {rgb_path}.")

            if depth_image is not None:
                print(f"Loaded Depth image {depth_path} successfully.")
            else:
                print(f"Failed to load Depth image {depth_path}.")

            # Assicurati che le immagini siano aggiunte solo se entrambe sono caricate correttamente
            if rgb_image is not None and depth_image is not None:
                rgb_images.append(rgb_image)
                depth_images.append(depth_image)
                timestamps.append(timestamp_rgb)
        else:
            print(f"No matching depth image for RGB timestamp {timestamp_rgb}.")

    return rgb_images, depth_images, timestamps
def rgbd_slam(rgb_images, depth_images, intrinsics):
    # Create the TSDF volume
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=4.0 / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    print("analyzing:", len(rgb_images), len(depth_images))

    # Initialize pose (identity matrix)
    pose = np.eye(4)
    poses = [pose]

    for i, (color, depth) in enumerate(zip(rgb_images, depth_images)):
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color),
            o3d.geometry.Image(depth),
            depth_trunc=4.0,
            convert_rgb_to_intensity=False)

        if i > 0:
            # Estimate the pose of the current frame
            prev_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(rgb_images[i - 1]),
                o3d.geometry.Image(depth_images[i - 1]),
                depth_trunc=4.0,
                convert_rgb_to_intensity=False)

            odo_init = o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()

            success, trans, info = o3d.pipelines.odometry.compute_rgbd_odometry(
                rgbd_image, prev_rgbd, intrinsics, np.eye(4), odo_init)

            if success:
                pose = np.dot(pose, trans)
                poses.append(pose)
                print(f"Frame {i}: Success, Transformation:\n{trans}")
            else:
                print(f"Frame {i}: Odometry failed")

        volume.integrate(rgbd_image, intrinsics, np.linalg.inv(pose))
        print(f"Integrated frame {i}, Pose:\n{pose}")

    # Extract the point cloud from the TSDF volume
    pcd = volume.extract_point_cloud()
    pcd.estimate_normals()

    if pcd is None or len(pcd.points) == 0:
        print("Point cloud extraction failed or resulted in an empty point cloud.")
        return None, poses

    o3d.visualization.draw_geometries([pcd], point_show_normal=True)

    return pcd, poses




def VISUAL_SLAM_ZED():
    rgb_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_img/RGB"
    depth_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/zed_img/Depth"
    #intrinsics_file = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/intrinsics_KAdepth.txt"
    output_ply = "output_RGBDVSLAM_ZED.ply"

    # Load images and intrinsics
    rgb_images, depth_images, timestamps = load_images(rgb_folder, depth_folder)
    print("loaded : ", len(rgb_images))
    intrinsics = load_intrinsics()

    pcd, poses = rgbd_slam(rgb_images, depth_images, intrinsics)

    if pcd is not None:
        output_ply = 'output_RGBDVSLAM_ZED.ply'
        o3d.io.write_point_cloud(output_ply, pcd)
    else:
        print("No point cloud to write.")




VISUAL_SLAM_ZED()