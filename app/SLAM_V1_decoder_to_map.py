import sys
import numpy as np
import open3d as o3d
import json
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




def integrate_angular_velocity(angular_velocity, timestamps):
    angles = [0]  # Start with an initial angle of 0
    for i in range(1, len(angular_velocity)):
        dt = timestamps[i] - timestamps[i - 1]
        angles.append(angles[-1] + angular_velocity[i] * dt)
    return angles

def integrate_linear_acceleration(acceleration, timestamps):
    velocities = [0]  # Start with an initial velocity of 0
    displacements = [0]  # Start with an initial displacement of 0
    for i in range(1, len(acceleration)):
        dt = timestamps[i] - timestamps[i - 1]
        velocities.append(velocities[-1] + acceleration[i] * dt)
        displacements.append(displacements[-1] + velocities[-1] * dt)
    return displacements

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

def create_trajectory_line_set(trajectory_points):
    points = o3d.utility.Vector3dVector(trajectory_points)
    lines = [[i, i + 1] for i in range(len(trajectory_points) - 1)]
    colors = [[1, 0, 0] for _ in range(len(lines))]  # Red color for trajectory lines
    line_set = o3d.geometry.LineSet()
    line_set.points = points
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def compute_displacements_and_euler_angles(imu_data, imu_timestamps):
    euler_angles = []
    displacements = []
    trajectory_points = []

    integrated_angular_velocity = np.array([0.0, 0.0, 0.0])  # Initial orientation (Euler angles)
    integrated_velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity
    integrated_displacement = np.array([0.0, 0.0, 0.0])  # Initial displacement

    last_time = imu_timestamps[0]

    for i, imu_msg in enumerate(imu_data):
        current_time = imu_timestamps[i]
        dt = current_time - last_time
        last_time = current_time

        # Extract angular velocity and linear acceleration
        angular_velocity = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
        linear_acceleration = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])

        # Integrate angular velocity to get Euler angles
        integrated_angular_velocity += angular_velocity * dt
        euler_angles.append(integrated_angular_velocity.copy())

        # Integrate linear acceleration to get displacement, taking orientation into account
        rotation = R.from_euler('xyz', integrated_angular_velocity).as_matrix()
        rotated_acceleration = rotation.dot(linear_acceleration)
        integrated_velocity += rotated_acceleration * dt
        integrated_displacement += integrated_velocity * dt

        displacements.append(integrated_displacement.copy())
        trajectory_points.append(integrated_displacement.copy())

    return euler_angles, displacements, trajectory_points

def interpolate_imu_data(pc_timestamps, imu_timestamps, euler_angles, displacements):
    interpolated_euler_angles = []
    interpolated_displacements = []

    for pc_time in pc_timestamps:
        idx = np.searchsorted(imu_timestamps, pc_time)
        if idx == 0:
            interpolated_euler_angles.append(euler_angles[0])
            interpolated_displacements.append(displacements[0])
        elif idx >= len(imu_timestamps):
            interpolated_euler_angles.append(euler_angles[-1])
            interpolated_displacements.append(displacements[-1])
        else:
            t1 = imu_timestamps[idx - 1]
            t2 = imu_timestamps[idx]
            t = pc_time

            weight1 = (t2 - t) / (t2 - t1)
            weight2 = (t - t1) / (t2 - t1)

            interpolated_euler_angles.append(weight1 * np.array(euler_angles[idx - 1]) + weight2 * np.array(euler_angles[idx]))
            interpolated_displacements.append(weight1 * np.array(displacements[idx - 1]) + weight2 * np.array(displacements[idx]))

    return interpolated_euler_angles, interpolated_displacements



def transform_pointcloud(pc, translation, rotation):
    # Apply the rotation
    r = R.from_euler('xyz', rotation, degrees=False)
    rotation_matrix = r.as_matrix()
    pc.rotate(rotation_matrix)
    # Apply the translation
    pc.translate(translation)
    return pc

def apply_transformations(pointclouds, euler_angles, displacements):
    transformed_pointclouds = []

    for i, pc in enumerate(pointclouds):
        translation = displacements[i]
        rotation = euler_angles[i]
        transformed_pc = transform_pointcloud(pc, translation, rotation)
        transformed_pointclouds.append(transformed_pc)

    return transformed_pointclouds



def downsample_pointcloud(pc, voxel_size=0.05):
    downsampled_pc = pc.voxel_down_sample(voxel_size)
    return downsampled_pc

def merge_pointclouds(transformed_pointclouds, voxel_size=0.15):
    combined = o3d.geometry.PointCloud()
    kk = 0
    for pc in transformed_pointclouds:
        kk += 1
        downsampled_pc = downsample_pointcloud(pc, voxel_size)
        combined += downsampled_pc
        print("merged:", kk)
    return combined



def visualize_pointcloud(pointcloud):
    o3d.visualization.draw_geometries([pointcloud])





##########################

def pointcloud2_to_o3d(msg):
    points = np.array(list(read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    return pc











def list_topics(bag_file):
    with rosbag.Bag(bag_file, 'r') as bag:
        info = bag.get_type_and_topic_info()
        topics = info.topics.keys()
        print("Topics presenti nel file bag:")
        for topic in topics:
            print(topic)



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



def save_trajectory_points(trajectory_points, filename="trajectory_points.json"):
    # Convert numpy arrays to lists
    trajectory_points_list = [point.tolist() for point in trajectory_points]
    with open(filename, 'w') as f:
        json.dump(trajectory_points_list, f)


def extract_imu_data_from_bag_xses(bag_file, topic_imu_xsens, output_file="imu_xsens_data.json"):
    imu_data = []

    hh  = 0
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_imu_xsens]):
            print(msg)
            hh += 1
            imu_entry = {
                'timestamp': t.to_sec(),
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            }
            imu_data.append(imu_entry)
            print("xses extraction:", hh)

    with open(output_file, 'w') as f:
        json.dump(imu_data, f)


def extract_and_save_gnss_data(bag_file, topic_gnss="/GNSS", output_file="gnss_data.json"):
    gnss_data = []

    gg = 0
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_gnss]):
            gg += 1
            gnss_entry = {
                'timestamp': t.to_sec(),
                'data': msg.data
            }
            gnss_data.append(gnss_entry)
            print("gnss extraction:", gg)

    with open(output_file, 'w') as f:
        json.dump(gnss_data, f)



def main():

    print("start")
    # Calcola e stampa il numero di immagini e gli FPS
    #calculate_fps_and_count(bag_file, rgb_topic, depth_topic)

    #stream_images_D455(bag_file, rgb_topic, depth_topic)


    bag_file = "/home/mmt-ben/Downloads/mid360_20240531_115939.bag"
    #bag_file = "/home/mmt-ben/Downloads/mid360_20240531_124831.bag"
    pc_topic = "/livox/lidar"
    topic_imu = "/livox/imu"

    topic_imu_xsens = "/imu/data"



    list_topics(bag_file)
    extract_and_save_gnss_data(bag_file, topic_gnss="/GNSS", output_file="gnss_data.json")

    #extract_imu_data_from_bag_xses(bag_file, topic_imu_xsens)






    pointclouds, pc_timestamps, imu_data, imu_timestamps = load_pointclouds_from_rosbag(bag_file, pc_topic, topic_imu)




    # Interpolate IMU data to get euler angles and displacements
    euler_angles, displacements, trajectory_points = compute_displacements_and_euler_angles(imu_data, imu_timestamps)




    interpolated_euler_angles, interpolated_displacements = interpolate_imu_data(pc_timestamps, imu_timestamps, euler_angles, displacements)
    save_trajectory_points(trajectory_points, "trajectory_points.json")


    sys.exit()


    save_imu_data_to_file(imu_data, imu_timestamps)





    # Apply transformations to point clouds
    transformed_pointclouds = apply_transformations(pointclouds, interpolated_euler_angles, interpolated_displacements)

    # Create trajectory line set
    trajectory_line_set = create_trajectory_line_set(trajectory_points)
    # Save the final point cloud
    o3d.io.write_point_cloud("merged_pointcloud.ply", transformed_pointclouds)


    # Visualize the transformed point clouds and trajectory
    o3d.visualization.draw_geometries(transformed_pointclouds + [trajectory_line_set])



    # Visualize the final point cloud
