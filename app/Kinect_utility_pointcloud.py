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

# Carica esplicitamente la libreria libk4a.so
ctypes.CDLL("libk4a.so")

# Stampa la variabile d'ambiente per verifica
print("LD_LIBRARY_PATH:", os.environ.get('LD_LIBRARY_PATH', 'Not set'))


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

                # Save depth image in int16 format
                depth_filename = os.path.join(output_depth_dir, f"depth_{pc_timestamp}.png")
                if not os.path.exists(depth_filename):
                    cv2.imwrite(depth_filename, depth_image.astype(np.uint16))
                else:
                    print(f"Depth image {depth_filename} already exists. Skipping...")

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

def extract_and_visualize(playback: PyK4APlayback, output_dir, timestamp_map, frame_map):
    """
    Extract point clouds and images from Kinect Azure MKV file and save them to the output directory.

    Args:
        playback (PyK4APlayback): Playback object for Kinect Azure MKV files.
        output_dir (str): Directory to save the output (point clouds and images).
        timestamp_map (dict): Dictionary to map sensor timestamps to global timestamps.
    """
    frame_count = 0

    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            break

        if capture is not None:
            point_cloud_data = capture.depth_point_cloud
            depth_image = capture.depth
            color_image = capture.color
            ir_image = capture.ir

            if point_cloud_data is not None and depth_image is not None and color_image is not None and ir_image is not None:
                sensor_timestamp = capture.depth_timestamp_usec

                # Convert sensor timestamp to global timestamp using frame number if necessary
                if sensor_timestamp in timestamp_map:
                    pc_timestamp = timestamp_map[sensor_timestamp]
                elif frame_count in frame_map:
                    pc_timestamp = frame_map[frame_count]
                else:
                    print(
                        f"Timestamp {sensor_timestamp} non trovato nella tabella di conversione e frame {frame_count} non trovato.")
                    continue

                # Save point cloud
                ply_filename = os.path.join(output_dir, 'pc', f"pointcloud_{pc_timestamp}.ply")
                if frame_count > 520:
                    if not os.path.exists(ply_filename):
                        # Save point cloud
                        points = point_cloud_data.reshape(-1, 3)
                        points = points[np.isfinite(points).all(axis=1)]
                        point_cloud = o3d.geometry.PointCloud()
                        point_cloud.points = o3d.utility.Vector3dVector(points)
                        o3d.io.write_point_cloud(ply_filename, point_cloud)
                    else:
                        print(f"Point cloud {ply_filename} already exists. Skipping...")

                # Save color image
                SAVE_RGB = 0
                if SAVE_RGB:
                    color_image_filename = os.path.join(output_dir, 'img', f"color_{pc_timestamp}.jpg")
                    cv2.imwrite(color_image_filename, color_image)
                    cv2.imshow("Color Image", color_image)

                SHOW_DEPTH = 0
                if SHOW_DEPTH:

                    # Visualize depth image
                    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                    depth_image_normalized = np.uint8(depth_image_normalized)
                    cv2.imshow("Depth Image", depth_image_normalized)

                # Visualize IR image
                ir_image_normalized = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
                ir_image_normalized = np.uint8(ir_image_normalized)
                cv2.imshow("IR Image", ir_image_normalized)

                # Visualize color image

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                frame_count += 1
                print(f"Frame {frame_count} processed")

    cv2.destroyAllWindows()


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



    extract_and_visualize(playback, output_dir, timestamp_map, frame_map)

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

def SLAM_V1():

    # Directory contenente le point cloud
    pointcloud_dir = "pc_ak"
    pointcloud_files = glob.glob(pointcloud_dir + "/*.ply")

    # Lista per salvare tutte le point cloud
    pointclouds = []

    iii = 0

    for filename in pointcloud_files:
        iii += 1
        print("loading :",iii)
        pcd = o3d.io.read_point_cloud(filename)
        pointclouds.append(pcd)

    print(f"Caricate {len(pointclouds)} point cloud.")

    # Allinea le point cloud
    aligned_pointclouds = [pointclouds[0]]

    for i in range(1, len(pointclouds)):

        print(f"Registrazione della point cloud {i} con la point cloud {i - 1}")
        trans = pairwise_registration(pointclouds[i], aligned_pointclouds[-1])
        pointclouds[i].transform(trans)
        aligned_pointclouds.append(pointclouds[i])

    print("Allineamento completato.")

    # Unisci tutte le point cloud allineate
    combined_pcd = aligned_pointclouds[0]
    kkkk = 0
    for pcd in aligned_pointclouds[1:]:

        kkkk += 1
        print("combined:",kkkk)
        combined_pcd += pcd

    # Salva la point cloud combinata
    o3d.io.write_point_cloud("combined_orb_pointcloud.ply", combined_pcd)

    print("Point cloud combinata salvata come combined_pointcloud.ply.")

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

def SLAM_V2():
    # Creazione della cartella per salvare le point cloud
    output_folder = "pc_ak"
    pointcloud_files = glob.glob(output_folder + "/*.ply")

    # Lista per salvare tutte le point cloud
    pointclouds = []

    for filename in pointcloud_files:
        pcd = o3d.io.read_point_cloud(filename)
        pointclouds.append(pcd)

    print(f"Caricate {len(pointclouds)} point cloud.")

    # Parametri
    voxel_size = 0.1  # Aumentare il voxel size per velocizzare il processo

    # Preprocessamento e registrazione delle point cloud
    aligned_pointclouds = [pointclouds[0]]
    current_transformation = np.identity(4)

    for i in range(1, len(pointclouds)):
        source = aligned_pointclouds[-1]
        target = pointclouds[i]

        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)

        print(f"Registrazione globale della point cloud {i}")
        global_result = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh, voxel_size)
        print(f"Raffinamento della registrazione della point cloud {i}")
        icp_result = refine_registration(source, target, global_result.transformation, voxel_size)

        target.transform(icp_result.transformation)
        aligned_pointclouds.append(target)

    print("Allineamento completato.")

    # Unisci tutte le point cloud allineate
    combined_pcd = aligned_pointclouds[0]
    for pcd in aligned_pointclouds[1:]:
        combined_pcd += pcd

    # Salva la point cloud combinata
    o3d.io.write_point_cloud("combined_pointcloud.ply", combined_pcd)

    print("Point cloud combinata salvata come combined_pointcloud.ply.")


def run_loam(input_ply_files, output_directory):
    # Percorsi degli eseguibili LOAM
    loam_dir = "/home/mmt-ben/loam_velodyne/build/devel/lib/loam_velodyne/"
    laserOdometry_executable = os.path.join(loam_dir, "laserOdometry")
    laserMapping_executable = os.path.join(loam_dir, "laserMapping")
    multiScanRegistration_executable = os.path.join(loam_dir, "multiScanRegistration")
    transformMaintenance_executable = os.path.join(loam_dir, "transformMaintenance")

    # Creazione directory di output
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    # Imposta l'ambiente ROS
    ros_env = os.environ.copy()
    ros_env["LD_LIBRARY_PATH"] = "/opt/ros/noetic/lib:" + ros_env.get("LD_LIBRARY_PATH", "")
    ros_env["PYTHONPATH"] = "/opt/ros/noetic/lib/python3/dist-packages:" + ros_env.get("PYTHONPATH", "")
    ros_env["CMAKE_PREFIX_PATH"] = "/opt/ros/noetic:" + ros_env.get("CMAKE_PREFIX_PATH", "")
    ros_env["ROS_PACKAGE_PATH"] = "/opt/ros/noetic/share:" + ros_env.get("ROS_PACKAGE_PATH", "")

    # Esecuzione di multiScanRegistration per ogni file PLY
    for ply_file in input_ply_files:
        output_file = os.path.join(output_directory, os.path.basename(ply_file).replace(".ply", ".pcd"))
        print(f"Processing {ply_file} -> {output_file}")
        subprocess.run([multiScanRegistration_executable, ply_file, output_file], env=ros_env)

    # Esecuzione di laserOdometry
    subprocess.run([laserOdometry_executable, output_directory], env=ros_env)

    # Esecuzione di laserMapping
    subprocess.run([laserMapping_executable, output_directory], env=ros_env)

    # Esecuzione di transformMaintenance
    subprocess.run([transformMaintenance_executable, output_directory], env=ros_env)


def SLAM_V3():
    # Lista di file PLY di input
    input_ply_files = glob.glob("/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak/*.ply")

    # Directory di output
    output_directory = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/loam_output"

    # Esegui LOAM
    run_loam(input_ply_files, output_directory)


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


def load_images(rgb_folder, depth_folder, skip_start=0, skip_end=4500):
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

    print("analizing:", len(rgb_images), len(depth_images))

    # Initialize pose (identity matrix)
    pose = np.eye(4)
    poses = [pose]

    for i, (color, depth) in enumerate(zip(rgb_images, depth_images)):

        # cv2.imshow("color",color)
        # cv2.imshow("depth", depth)
        #
        # cv2.waitKey(0)  # Wait for a key press



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

    # Extract the mesh from the TSDF volume
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    return mesh, poses


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



#process_mkv_4_RGBD()
#process_mkv()
#SLAM_V1()
#SLAM_V2()

#funziona standard ma orribile
#SLAM_V4()

#SLAM_V3()

VISULA_SLAM()
