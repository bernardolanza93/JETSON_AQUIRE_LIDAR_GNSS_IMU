from custom_slam_doc import *
from pyk4a import PyK4APlayback, Config, PyK4A
from pyk4a import PyK4APlayback, CalibrationType
import open3d as o3d
import numpy as np
import os
import pandas as pd
import cv2
import matplotlib
import ctypes
import pyk4a
from matrix_utilities import *







# Carica esplicitamente la libreria libk4a.so
ctypes.CDLL("libk4a.so")

# Stampa la variabile d'ambiente per verifica
print("LD_LIBRARY_PATH:", os.environ.get('LD_LIBRARY_PATH', 'Not set'))


def load_k4a_library():
    k4a = ctypes.CDLL("/usr/local/lib/libk4a.so")
    return k4a


def process_mkv(input_file, output_dir, start_index, timestamp_file):
    """
     Main function to handle directory setup and initiate processing of MKV files.
     """

    os.makedirs(output_dir, exist_ok=True)


    timestamp_map, frame_map = load_timestamp_conversion(timestamp_file)

    playback = PyK4APlayback(input_file)
    playback.open()



    extract_and_visualize(playback, output_dir, timestamp_map, frame_map, start_index)

    playback.close()

def load_timestamp_conversion(file_path):
    """Load the timestamp conversion table."""
    timestamp_conversion = pd.read_csv(file_path)
    timestamp_map = dict(zip(timestamp_conversion['sensor timestamp'], timestamp_conversion['pc timestamp']))
    frame_map = dict(zip(timestamp_conversion['frame'], timestamp_conversion['pc timestamp']))
    return timestamp_map, frame_map


def extract_and_visualize(playback, output_dir, timestamp_map, frame_map, start_index):

    frame_count = 0

    points_rejected = []

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



                if frame_count > start_index:

                    point_cloud_data = capture.depth_point_cloud
                    points = point_cloud_data.reshape(-1, 3)
                    ini_len  =len(points)
                    points = points[np.isfinite(points).all(axis=1)]
                    capture._color = cv2.cvtColor(cv2.imdecode(capture.color, cv2.IMREAD_COLOR), cv2.COLOR_BGR2BGRA)
                    capture._color_format = pyk4a.ImageFormat.COLOR_BGRA32
                    transformed_color_img = cv2.cvtColor(capture.transformed_color, cv2.COLOR_BGR2RGB)
                    colors = transformed_color_img.reshape(-1, 3) / 255
                    # Remove invalid points
                    condition = (points[:, 0] < 1) & (points[:, 1] < 1) & (points[:, 2] < 1)
                    points = points[~condition]
                    colors = colors[~condition]
                    rejected = ini_len-len(points)
                    print("total:", ini_len, "rejected:", rejected)
                    points_rejected.append(rejected)

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
    print("mean rejected ", np.mean(points_rejected))
    playback.close()




