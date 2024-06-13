import pyrealsense2 as rs
import cv2
import numpy as np
import os
import time
import csv
from datetime import datetime, timezone


## CODE PART FOR RUNNING REALTIME THE D455 and acquire config

def calculate_and_save_intrinsics(intrinsics):
    """
    Function to calculate and save the camera intrinsics to a CSV file.
    Args:
        intrinsics: The intrinsics object containing camera calibration data.
    """
    title = "intrinsics.csv"

    # Check if the file exists, if not, create it and write the intrinsics data.
    if not os.path.exists(title):
        data_int = [intrinsics.width, intrinsics.height, intrinsics.ppx, intrinsics.ppy, intrinsics.fx, intrinsics.fy, intrinsics.model, intrinsics.coeffs]
        with open(title, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data_int)
        print("new file intrinsics written")

# Create a context object for the RealSense pipeline
ctx = rs.context()
pipeline = rs.pipeline(ctx)
config = rs.config()

device_aviable = {}

# Check if any devices are connected
if len(ctx.devices) > 0:
    for d in ctx.devices:
        # Get the device name and serial number
        device = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        model = str(device.split(' ')[-1])

        # Store the device information
        device_aviable[model] = [serial, device]

        print('Found device: ', device_aviable)
else:
    print("No Intel Device connected")

# Get the serial number for the D455 device
seriald455 = str(device_aviable['D455'][0])

# Enable the color and depth streams for the device
config.enable_device(seriald455)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# Align the depth frames to the color frames
align_to = rs.stream.color
align = rs.align(align_to)

pipeline.start(config)

frame_c = 0
CAPTURE = 1

while CAPTURE:
    try:
        # Wait for a new set of frames from the camera
        frames = pipeline.wait_for_frames()
        frame_c += 1

    except Exception as e:
        print("PIPELINE error: %s", str(e))

    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Save the intrinsics of the first few frames
    if frame_c < 3:
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        calculate_and_save_intrinsics(depth_intrin)
        print("DEPTH",depth_intrin)
        print("COLOR", color_intrin)

# Post processing part to handle datatype

def obtain_intrinsics():
    """
    Function to read intrinsics from a CSV file and return an intrinsics object.
    Returns:
        intrinsics: The intrinsics object populated with the data from the CSV file.
    """
    intrinsics = rs.intrinsics()
    with open("intrinsics.csv", "r") as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            if i == 0:
                intrinsics.width = int(line[0])
                intrinsics.height = int(line[1])
                intrinsics.ppx = float(line[2])
                intrinsics.ppy = float(line[3])
                intrinsics.fx = float(line[4])
                intrinsics.fy = float(line[5])

                if str(line[6]) == "distortion.inverse_brown_conrady":
                    intrinsics.model = rs.distortion.inverse_brown_conrady
                else:
                    print("not recognized this string for model: ", str(line[6]))
                    intrinsics.model = rs.distortion.inverse_brown_conrady

                listm = line[7].split(",")

                new_list = []
                for element in listm:
                    element = element.replace("[", "").replace(" ", "").replace("]", "")
                    new_list.append(float(element))

                intrinsics.coeffs = new_list

    return intrinsics

def save_data(frames, rs_data_path):
    """
    Save frames data as point cloud with additional information about RGB and infrared values.
    Args:
        frames: The frame data captured from the sensor.
        rs_data_path (str): Path to save the point cloud data.
    """
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    infrared_frame = frames.first(rs.stream.infrared)

    if not color_frame or not depth_frame or not infrared_frame:
        print("INVALID FRAME")
        return

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    ir_image = np.asanyarray(infrared_frame.get_data())

    points, colors, ir_values = [], [], []
    for v in range(depth_image.shape[0]):
        for u in range(depth_image.shape[1]):
            distance = float(depth_image[v, u])
            result = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], distance)
            Y = int(-result[1])
            X = int(-result[0])
            Z = int(result[2])
            if Z == 0: continue  # Ignore zero depth values
            IR = ir_image[v, u]
            ir_values.append(IR)
            points.append((X, Y, Z))
            colors.append(color_image[v, u])

    points = np.array(points)
    colors = np.array(colors)
    ir_values = np.array(ir_values).reshape(-1, 1)

    # Combine points and colors
    combined_pc = np.hstack((points, colors))
    combined_pc_ir = np.hstack((combined_pc, ir_values))

    # Ensure the directories exist
    if not os.path.exists(os.path.join(rs_data_path, 'RGB')):
        os.makedirs(os.path.join(rs_data_path, 'RGB'))
    if not os.path.exists(os.path.join(rs_data_path, 'DEPTH')):
        os.makedirs(os.path.join(rs_data_path, 'DEPTH'))
    if not os.path.exists(os.path.join(rs_data_path, 'IR')):
        os.makedirs(os.path.join(rs_data_path, 'IR'))
    if not os.path.exists(os.path.join(rs_data_path, 'PC')):
        os.makedirs(os.path.join(rs_data_path, 'PC'))
    #
    # timestamp = frames.get_timestamp()
    # formated_date = timestamp_to_datetime(timestamp)
    # timestampt_path = int(timestamp)
    #
    # # File names: DATA_TYPE_DATE_TIME-TIMESTAMP.format
    # filename_rgb = os.path.join(rs_data_path, 'RGB',
    #                             f'rgb_D455_{formated_date}-{timestampt_path}.png')
    # filename_depth = os.path.join(rs_data_path, 'DEPTH',
    #                               f'depth_D455_{formated_date}-{timestampt_path}.npy')
    # filename_ir = os.path.join(rs_data_path, 'IR',
    #                            f'ir_D455_{formated_date}-{timestampt_path}.npy')
    # filename_pc = os.path.join(rs_data_path, 'PC',
    #                            f'pc_D455_{formated_date}-{timestampt_path}.txt')
    #
    # # Save captures
    # cv2.imwrite(filename_rgb, color_image)
    # np.save(filename_depth, depth_image)
    # np.save(filename_ir, ir_image)
    # np.savetxt(filename_pc, combined_pc_ir, fmt='%f, %f, %f, %d, %d, %d, %d')



