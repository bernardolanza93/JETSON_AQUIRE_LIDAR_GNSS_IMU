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
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('agg')
import glob
import open3d as o3d
import numpy as np
import os
from pyk4a import PyK4APlayback, CalibrationType
from pyk4a import PyK4A
from pyk4a import Config, PyK4APlayback
from pyk4a import PyK4APlayback, Config, PyK4A

from ctypes import c_int, c_void_p, c_char_p, POINTER




INITIAL_LIMIT = 397

# Carica esplicitamente la libreria libk4a.so
ctypes.CDLL("libk4a.so")

# Stampa la variabile d'ambiente per verifica
print("LD_LIBRARY_PATH:", os.environ.get('LD_LIBRARY_PATH', 'Not set'))


def load_k4a_library():
    k4a = ctypes.CDLL("/usr/local/lib/libk4a.so")
    return k4a

def transform_color_to_depth(k4a, transformation_handle, depth_image, color_image):
    # Define the k4a_image_t type
    k4a_image_t = ctypes.c_void_p

    # Create k4a_image_t instances for depth and color images
    depth_image_t = k4a_image_t(depth_image.ctypes.data_as(ctypes.c_void_p))
    color_image_t = k4a_image_t(color_image.ctypes.data_as(ctypes.c_void_p))

    # Create transformed color image placeholder
    transformed_color_image = np.zeros_like(color_image)
    transformed_color_image_t = k4a_image_t(transformed_color_image.ctypes.data_as(ctypes.c_void_p))

    # Perform the transformation
    k4a.k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                       depth_image_t,
                                                       transformed_color_image_t)
    return transformed_color_image



def RGBD_extraction(playback: PyK4APlayback, output_rgb_dir: str, output_depth_dir: str, output_intrinsics_file: str, timestamp_map: dict, frame_map: dict):
    """
    Extract RGB and depth images from Kinect Azure MKV file and save them to the output directories.

    Args:
        playback (PyK4APlayback): Playback object for Kinect Azure MKV files.
        output_rgb_dir (str): Directory to save the RGB images.
        output_depth_dir (str): Directory to save the depth images.
        output_intrinsics_file (str): File to save the camera intrinsics.
        timestamp_map (dict): Dictionary to map sensor timestamps to global timestamps.
        frame_map (dict): Dictionary to map frame counts to global timestamps.
    """
    frame_count = 0
    intrinsics_saved = False


    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            break


        if capture is not None:
            depth_image = capture.depth
            #color_image = cv2.cvtColor(cv2.imdecode(capture.color, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            color_image = cv2.imdecode(capture.color, cv2.IMREAD_COLOR)

            if depth_image is not None and color_image is not None:
                sensor_timestamp = capture.depth_timestamp_usec

                if frame_count > INITIAL_LIMIT:
                    print("+save")

                    # Convert sensor timestamp to global timestamp using frame number if necessary
                    if sensor_timestamp in timestamp_map:
                        pc_timestamp = timestamp_map[sensor_timestamp]
                    elif frame_count in frame_map:
                        pc_timestamp = frame_map[frame_count]
                    else:
                        print(f"Timestamp {sensor_timestamp} non trovato nella tabella di conversione e frame {frame_count} non trovato.")
                        continue

                    # Save RGB image as PNG
                    rgb_filename = os.path.join(output_rgb_dir, f"color_{pc_timestamp}.png")
                    if not os.path.exists(rgb_filename):
                        cv2.imwrite(rgb_filename, color_image)
                    else:
                        print(f"RGB image {rgb_filename} already exists. Skipping...")

                    # # Save depth image in int16 format
                    # depth_filename = os.path.join(output_depth_dir, f"depth_{pc_timestamp}.png")
                    # if not os.path.exists(depth_filename):
                    #     cv2.imwrite(depth_filename, depth_image.astype(np.uint16))
                    # else:
                    #     print(f"Depth image {depth_filename} already exists. Skipping...")

                    # Save intrinsics once
                    if not intrinsics_saved:
                        calibration = playback.calibration
                        intrinsics = calibration.get_camera_matrix(CalibrationType.COLOR)
                        distortion = calibration.get_distortion_coefficients(CalibrationType.COLOR)
                        with open(output_intrinsics_file, 'w') as f:
                            f.write(f"Camera Intrinsics:\n{intrinsics}\n")
                            f.write(f"Distortion Coefficients:\n{distortion}\n")
                        intrinsics_saved = True

                frame_count += 1
                print(f"Frame {frame_count} processed")



def load_timestamp_conversion(file_path):
    """Load the timestamp conversion table."""
    timestamp_conversion = pd.read_csv(file_path)
    timestamp_map = dict(zip(timestamp_conversion['sensor timestamp'], timestamp_conversion['pc timestamp']))
    frame_map = dict(zip(timestamp_conversion['frame'], timestamp_conversion['pc timestamp']))
    return timestamp_map, frame_map



def extract_and_visualize(playback, output_dir, timestamp_map, frame_map):

    frame_count = 0

    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            break

        if capture is not None:
            depth_image = capture.depth
            color_image = cv2.imdecode(np.frombuffer(capture.color, np.uint8), cv2.IMREAD_COLOR)

            if depth_image is not None and color_image is not None:
                sensor_timestamp = capture.depth_timestamp_usec

                # Convert sensor timestamp to global timestamp using frame number if necessary
                if sensor_timestamp in timestamp_map:
                    pc_timestamp = timestamp_map[sensor_timestamp]
                elif frame_count in frame_map:
                    pc_timestamp = frame_map[frame_count]
                else:
                    print(f"Timestamp {sensor_timestamp} non trovato.")
                    continue

                # # Obtain the calibration
                # calibration = playback.calibration
                # intrinsics = calibration.get_camera_matrix(CalibrationType.DEPTH)
                # fx, fy, cx, cy = intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]
                # height, width = depth_image.shape
                #
                # x = np.linspace(0, width - 1, width)
                # y = np.linspace(0, height - 1, height)
                # xv, yv = np.meshgrid(x, y)
                #
                # z = depth_image / 1000.0  # Convert from mm to meters
                # x = (xv - cx) * z / fx
                # y = (yv - cy) * z / fy



                if frame_count > INITIAL_LIMIT:

                    point_cloud_data = capture.depth_point_cloud
                    points = point_cloud_data.reshape(-1, 3)
                    points = points[np.isfinite(points).all(axis=1)]
                    capture._color = cv2.cvtColor(cv2.imdecode(capture.color, cv2.IMREAD_COLOR), cv2.COLOR_BGR2BGRA)
                    capture._color_format = pyk4a.ImageFormat.COLOR_BGRA32
                    transformed_color_img = cv2.cvtColor(capture.transformed_color, cv2.COLOR_BGR2RGB)
                    colors = transformed_color_img.reshape(-1, 3) / 255
                    # Remove invalid points
                    condition = (points[:, 0] < 1) & (points[:, 1] < 1) & (points[:, 2] < 1)
                    points = points[~condition]
                    colors = colors[~condition]
                    #Create and save the point cloud

                    SAVE = 1
                    if SAVE:
                        point_cloud = o3d.geometry.PointCloud()
                        point_cloud.points = o3d.utility.Vector3dVector(points)
                        point_cloud.colors = o3d.utility.Vector3dVector(colors)
                        ply_filename = os.path.join(output_dir, f"pointcloud_{pc_timestamp}.ply")
                        o3d.io.write_point_cloud(ply_filename, point_cloud)

                    if 0: #RENDERING
                        # cv2.imshow("ff", transformed_color_img)
                        # cv2.waitKey(0)
                        #
                        # cv2.destroyAllWindows()

                        point_cloud = o3d.geometry.PointCloud()
                        point_cloud.points = o3d.utility.Vector3dVector(points)
                        point_cloud.colors = o3d.utility.Vector3dVector(colors)
                        o3d.visualization.draw_geometries([point_cloud])


                frame_count += 1
                print(f"Frame {frame_count} processed")

    playback.close()


def process_mkv_4_RGBD():
    """
     Main function to handle directory setup and initiate processing of MKV files.
     """
    input_file = '/home/mmt-ben/Downloads/20240531_120721.mkv'  # Path to the MKV file
    timestamp_file = "/home/mmt-ben/Downloads/20240531_120721_timestamps.csv"  # Path to the timestamp conversion file
    output_dir_rgb = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/RGBKA"  # Directory to save the results
    output_dir_d = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/DKA"  # Directory to save the results
    intrinsics_KA = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/intrinsics_KAdepth.txt"


    os.makedirs(output_dir_rgb, exist_ok=True)
    os.makedirs(output_dir_d, exist_ok=True)


    timestamp_map, frame_map = load_timestamp_conversion(timestamp_file)

    playback = PyK4APlayback(input_file)
    playback.open()



    RGBD_extraction(playback, output_dir_rgb, output_dir_d, intrinsics_KA, timestamp_map, frame_map)

    playback.close()


def process_mkv():
    """
     Main function to handle directory setup and initiate processing of MKV files.
     """
    input_file = '/home/mmt-ben/Downloads/20240531_120721.mkv'  # Path to the MKV file
    timestamp_file = "/home/mmt-ben/Downloads/20240531_120721_timestamps.csv"  # Path to the timestamp conversion file
    output_dir = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak"  # Directory to save the results

    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'img'), exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'pc'), exist_ok=True)

    timestamp_map, frame_map = load_timestamp_conversion(timestamp_file)

    playback = PyK4APlayback(input_file)
    playback.open()



    extract_and_visualize(playback, os.path.join(output_dir, 'pc'), timestamp_map, frame_map)

    playback.close()

def pairwise_registration(source, target):
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance=0.05,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        init=np.eye(4))

    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance=0.01,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        init=icp_coarse.transformation)

    return icp_fine.transformation


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def refine_registration(source, target, init_transformation, voxel_size):
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    distance_threshold = voxel_size * 0.4
    icp_criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)  # Ridurre il numero di iterazioni
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=icp_criteria)
    return result


def load_and_preprocess_point_clouds(folder_path, voxel_size, min_neighbors=20, std_ratio=2.0, x_threshold=-1250):
    point_clouds = []
    iiii = 0
    for filename in sorted(os.listdir(folder_path)):
        if filename.endswith(".ply"):
            iiii += 1
            print("filtering:", iiii)
            pcd = o3d.io.read_point_cloud(os.path.join(folder_path, filename))
            # Rimuovere i punti con x < x_threshold
            points = np.asarray(pcd.points)
            mask = points[:, 0] > x_threshold
            pcd_filtered = pcd.select_by_index(np.where(mask)[0])
            # Applicare un downsampling iniziale
            pcd_downsampled = pcd_filtered.voxel_down_sample(voxel_size)
            # Filtrare i punti solitari
            cl, ind = pcd_downsampled.remove_statistical_outlier(nb_neighbors=min_neighbors, std_ratio=std_ratio)
            pcd_filtered = pcd_downsampled.select_by_index(ind)
            point_clouds.append(pcd_filtered)
    return point_clouds

def apply_icp_incremental(pcds, threshold=0.02, show_visualization=False):
    pcd_combined = pcds[0]
    transformation_matrices = [np.identity(4)]
    for i in range(1, len(pcds)):
        print("proc:", i)
        pcd_current = pcds[i]
        trans_init = np.identity(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd_combined, pcd_current, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        transformation_matrices.append(reg_p2p.transformation)
        # Trasforma la point cloud corrente secondo la matrice di trasformazione ottenuta
        pcd_current.transform(reg_p2p.transformation)
        # Combina la point cloud trasformata con la point cloud combinata corrente
        pcd_combined += pcd_current
        if show_visualization:
            print(f"Iteration {i}: Showing combined point cloud")
            o3d.visualization.draw_geometries([pcd_combined])
    return pcd_combined, transformation_matrices


def SLAM_V4():
    folder_path = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak/pc"
    #folder_path = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_zed_reg"

    voxel_size = 0.001  # Dimensione del voxel per il downsampling
    min_neighbors = 80  # Numero minimo di vicini per mantenere un punto
    std_ratio = 0.8  # Rapporto di deviazione standard per il filtraggio statistico
    show_visualization = True  # Imposta su False per disabilitare la visualizzazione
    pcds = load_and_preprocess_point_clouds(folder_path, voxel_size, min_neighbors, std_ratio)
    combined_pcd, transformations = apply_icp_incremental(pcds)
    o3d.io.write_point_cloud("combined_kinect_point_cloud.ply", combined_pcd)


def load_images(rgb_folder, depth_folder, skip_start=2200, skip_end=2600):
    rgb_images = []
    depth_images = []
    timestamps = sorted([f.split('_')[1].split('.png')[0] for f in os.listdir(rgb_folder) if f.endswith('.png')])

    # Apply the skip to both start and end
    timestamps = timestamps[skip_start:len(timestamps) - skip_end]
    i = 0
    for ts in timestamps:
        i += 1
        print("ec", i)
        rgb_path = os.path.join(rgb_folder, f"color_{ts}.png")
        depth_path = os.path.join(depth_folder, f"depth_{ts}.png")


        rgb_image = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # Load as uint16







        if rgb_image is not None and depth_image is not None:
            # Ensure the images are in the correct format
            if rgb_image.dtype != np.uint8:
                rgb_image = rgb_image.astype(np.uint8)
            if depth_image.dtype != np.uint16:
                depth_image = depth_image.astype(np.uint16)

            # Resize the RGB image to match the depth image resolution
            rgb_image = cv2.resize(rgb_image, (depth_image.shape[1], depth_image.shape[0]))

            rgb_images.append(rgb_image)
            depth_images.append(depth_image)

    return rgb_images, depth_images, timestamps


def load_intrinsics():
    # Hardcoded intrinsics based on the provided file
    intrinsics_matrix = np.array([[602.24353027,   0.,          641.01550293],
                                  [  0.,         602.10705566, 365.72381592],
                                  [  0.,           0.,           1.        ]], dtype=np.float64)
    scale_x = 640 / 1280
    scale_y = 576 / 720
    intrinsics_matrix[0, 0] *= scale_x
    intrinsics_matrix[1, 1] *= scale_y
    intrinsics_matrix[0, 2] *= scale_x
    intrinsics_matrix[1, 2] *= scale_y
    return o3d.camera.PinholeCameraIntrinsic(
        width=640, height=576,
        fx=intrinsics_matrix[0, 0], fy=intrinsics_matrix[1, 1], cx=intrinsics_matrix[0, 2], cy=intrinsics_matrix[1, 2]
    )


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
        # Convert depth image to o3d.geometry.Image
        depth_image = o3d.geometry.Image(depth)



        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color),
            depth_image,
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

        # Debug: Check volume data after integration
        tsdf_pcd = volume.extract_voxel_point_cloud()
        if tsdf_pcd is None or len(tsdf_pcd.points) == 0:
            print(f"Voxel point cloud extraction failed for frame {i} or resulted in an empty point cloud.")
        else:
            print(f"Voxel point cloud extracted for frame {i}, contains {len(tsdf_pcd.points)} points.")

    # Extract the mesh from the TSDF volume
    mesh = volume.extract_triangle_mesh()
    if mesh is None or len(mesh.vertices) == 0:
        print("Final mesh extraction failed or resulted in an empty mesh.")
        return None, poses

    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    # Convert the mesh to a point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.normals = mesh.vertex_normals

    return pcd, poses


def VISULA_SLAM():
    rgb_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/RGBKA"
    depth_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/DKA"
    intrinsics_file = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/intrinsics_KAdepth.txt"
    output_ply = "output_RGBDVSLAM.ply"

    # Load images and intrinsics
    rgb_images, depth_images, timestamps = load_images(rgb_folder, depth_folder)
    intrinsics = load_intrinsics()

    # Run RGB-D SLAM
    mesh = rgbd_slam(rgb_images, depth_images, intrinsics)

    # Save the resulting mesh
    o3d.io.write_triangle_mesh(output_ply, mesh)
    print(f"Saved point cloud to {output_ply}")






if __name__ == "__main__":


    #process_mkv_4_RGBD()
    process_mkv()
    #SLAM_V1()
    #SLAM_V2()ù



    #funziona standard ma orribile
    #SLAM_V4()

    #SLAM_V3()

    #VISULA_SLAM()
