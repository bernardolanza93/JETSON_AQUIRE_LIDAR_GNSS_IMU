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


def remove_isolated_points(pcd, nb_neighbors=10, radius=80):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    num_removed_points = len(pcd.points) - len(pcd_cleaned.points)
    print("OUTLIER REMOVED ", num_removed_points, " POINTS")
    return pcd_cleaned

def resize_and_rotate(image, scale_percent):
    # Read the image


    # Calculate the new dimensions
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    new_dim = (width, height)

    # Resize the image
    resized_image = cv2.resize(image, new_dim, interpolation=cv2.INTER_AREA)

    # Rotate the image 90 degrees clockwise
    rotated_image = cv2.rotate(resized_image, cv2.ROTATE_90_CLOCKWISE)

    return rotated_image

def show_rgb_video_mkv(mkv_file):
    playback = PyK4APlayback(mkv_file)
    playback.open()
    frame_count = 0
    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            break


        if capture is not None:
            try:
                color_image = cv2.imdecode(capture.color, cv2.IMREAD_COLOR)

                processed_image = resize_and_rotate(color_image, 40)
                cv2.imshow("color",processed_image)
                # Wait for key press and break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                frame_count += 1
                print("frame:", frame_count)
            except Exception as e:
                print("ERROR! SKIPPED:",e)

    playback.close()



def load_k4a_library():
    k4a = ctypes.CDLL("/usr/local/lib/libk4a.so")
    return k4a


def process_mkv(input_file, output_dir, start_index, timestamp_file, end_index):
    """
     Main function to handle directory setup and initiate processing of MKV files.
     """

    os.makedirs(output_dir, exist_ok=True)


    timestamp_map, frame_map = load_timestamp_conversion(timestamp_file)

    playback = PyK4APlayback(input_file)
    playback.open()



    extract_and_visualize(playback, output_dir, timestamp_map, frame_map, start_index,end_index)

    playback.close()

def load_timestamp_conversion(file_path):
    """Load the timestamp conversion table."""
    timestamp_conversion = pd.read_csv(file_path)
    timestamp_map = dict(zip(timestamp_conversion['Kinect Timestamp'], timestamp_conversion['System Timestamp']))
    frame_map = dict(zip(timestamp_conversion['Frame Number'], timestamp_conversion['System Timestamp']))
    return timestamp_map, frame_map

def extract_and_visualize(playback, output_dir, timestamp_map, frame_map, start_index, end_index):

    frame_count = 0

    points_rejected = []

    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            print("no capture!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            break

        if capture is not None:
            if frame_count > start_index and frame_count < end_index:
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


                    ply_filename = os.path.join(output_dir, f"pointcloud_{pc_timestamp}.ply")
                    if not os.path.exists(ply_filename):


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
                        print( "REMAINS:" , ini_len-rejected, "/", ini_len)
                        points_rejected.append(rejected)

                        #Create and save the point cloud

                        SAVE = 1
                        if SAVE:
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(points)
                            point_cloud.colors = o3d.utility.Vector3dVector(colors)
                            filtered_point_cloud = o3d.geometry.PointCloud()
                            filtered_point_cloud.points = o3d.utility.Vector3dVector(points)
                            filtered_point_cloud.colors = o3d.utility.Vector3dVector(colors)

                            remove_noise = 1
                            if remove_noise:
                                filtered_point_cloud = remove_isolated_points(filtered_point_cloud)
                            if len(filtered_point_cloud.points) > 0:
                                # Salva la point cloud filtrata se contiene punti
                                o3d.io.write_point_cloud(ply_filename, filtered_point_cloud)
                                print(f"SAVED")
                            else:
                                # Salva la point cloud originale se la filtrata non contiene punti
                                o3d.io.write_point_cloud(ply_filename, point_cloud)
                                print(f"ZERO POINTS {ply_filename} due to filtered point cloud having zero points")


            frame_count += 1
            print(f"Frame {frame_count} processed")
    print("mean rejected ", np.mean(points_rejected))




def inspect_mkv_file(file):
    show_image = 0
    playback = PyK4APlayback(file)
    playback.open()
    frame_count = 0
    while True:
        try:
            capture = playback.get_next_capture()
        except EOFError:
            break

        if capture is not None:
            try:
                print(dir(capture))
                if show_image:
                    color_image = cv2.imdecode(capture.color, cv2.IMREAD_COLOR)

                    processed_image = resize_and_rotate(color_image, 40)
                    cv2.imshow("color", processed_image)
                    # Wait for key press and break the loop if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    frame_count += 1
                    print("frame:", frame_count)
            except Exception as e:
                print("ERROR! SKIPPED:", e)

    playback.close()




mkv_file = "/home/mmt-ben/Downloads/20240716_113102_master.mkv"
#inspect_mkv_file(mkv_file)

