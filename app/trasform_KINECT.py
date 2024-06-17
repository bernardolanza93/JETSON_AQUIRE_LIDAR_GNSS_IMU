import json
import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import open3d as o3d
import os
import re
from scipy.spatial import KDTree


def plot_gnss_trajectory(json_file_path, scale_factor=100000):
    # Load GNSS data
    with open(json_file_path, 'r') as file:
        gnss_data = json.load(file)

    # Function to parse NMEA sentences and extract GNSS info
    def parse_nmea(nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return None, None, None, None  # Incomplete data
        try:
            timestamp = float(parts[1])
            latitude = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                latitude = -latitude
            longitude = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                longitude = -longitude
            altitude = float(parts[9])
            return timestamp, latitude, longitude, altitude
        except ValueError:
            return None, None, None, None  # Invalid data

    # Extract GNSS info with timestamp
    gnss_info = []
    for entry in gnss_data:
        ts, lat, lon, alt = parse_nmea(entry['data'].split('_')[1])
        if ts is not None and lat is not None and lon is not None and alt is not None:
            gnss_info.append({
                'timestamp': entry['timestamp'],
                'gnss_timestamp': ts,
                'latitude': lat,
                'longitude': lon,
                'altitude': alt
            })

    if not gnss_info:
        print("No valid GNSS data found.")
        return

    # Convert to numpy array for easier manipulation
    gnss_array = np.array(
        [[item['timestamp'], item['latitude'], item['longitude'], item['altitude']] for item in gnss_info])

    # Choose a reference point for the local coordinate system
    reference_latitude = gnss_array[0, 1]
    reference_longitude = gnss_array[0, 2]

    # Function to convert GNSS coordinates to a local coordinate system


    # Convert GNSS coordinates to local coordinates
    local_coordinates = np.array(
        [gnss_to_local(lat, lon, reference_latitude, reference_longitude, scale_factor) for _, lat, lon, _ in
         gnss_array])

    # Plotting the GNSS trajectory in local coordinates
    plt.figure(figsize=(10, 8))
    plt.plot(local_coordinates[:, 0], local_coordinates[:, 1], marker='o', linestyle='-', color='b')
    plt.title('GNSS Trajectory in Local Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.show()

    return gnss_array, local_coordinates, reference_latitude, reference_longitude, scale_factor

    # Function to convert GNSS coordinates to a local coordinate system
def gnss_to_local(latitude, longitude, reference_latitude, reference_longitude, scale_factor):
    x = (longitude - reference_longitude) * scale_factor
    y = (latitude - reference_latitude) * scale_factor
    return x, y



def interpolate_gnss_data(gnss_array, local_coordinates):
    # Interpolate latitude, longitude, and altitude
    timestamps = gnss_array[:, 0]
    latitudes = gnss_array[:, 1]
    longitudes = gnss_array[:, 2]
    altitudes = gnss_array[:, 3]

    interp_lat = interp1d(timestamps, latitudes, kind='linear', fill_value='extrapolate')
    interp_lon = interp1d(timestamps, longitudes, kind='linear', fill_value='extrapolate')
    interp_alt = interp1d(timestamps, altitudes, kind='linear', fill_value='extrapolate')

    return interp_lat, interp_lon, interp_alt


def get_transformation_matrix(lat, lon, alt, ref_lat, ref_lon, scale_factor):
    y,x = gnss_to_local(lat, lon, ref_lat, ref_lon, scale_factor)
    # Constructing a transformation matrix:
    # Here we assume a simple translation matrix. You may need to extend this to a full transformation matrix if rotation is involved.
    transformation_matrix = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, alt],
        [0, 0, 0, 1]
    ])
    return transformation_matrix


def associate_pointclouds_with_gnss(pointcloud_dir, interp_lat, interp_lon, interp_alt, ref_lat, ref_lon, scale_factor):
    transformation_matrices = []
    pointcloud_files = [f for f in os.listdir(pointcloud_dir) if f.endswith('.ply')]
    pointcloud_timestamps = []

    # Extract timestamps from filenames
    for file in pointcloud_files:
        match = re.search(r'(\d+\.\d+)', file)
        if match:
            timestamp = float(match.group(1))
            pointcloud_timestamps.append(timestamp)

    # Sort files and timestamps together
    pointcloud_files, pointcloud_timestamps = zip(*sorted(zip(pointcloud_files, pointcloud_timestamps)))

    # Associate transformation matrices
    dict_transformation ={}
    for pc_timestamp in pointcloud_timestamps:
        lat = interp_lat(pc_timestamp)
        lon = interp_lon(pc_timestamp)
        alt = interp_alt(pc_timestamp)
        transformation_matrix = get_transformation_matrix(lat, lon, alt, ref_lat, ref_lon, scale_factor)
        transformation_matrices.append(transformation_matrix)
        dict_transformation[str(pc_timestamp)] = transformation_matrix

    return pointcloud_files, transformation_matrices , dict_transformation


def plot_pointcloud_translations(pre_icp_matrices, post_icp_matrices):
    pre_translations = np.array([matrix[:2, 3] for matrix in pre_icp_matrices])
    pre_orientations = np.array([matrix[:2, 0] for matrix in pre_icp_matrices])

    post_translations = np.array([matrix[:2, 3] for matrix in post_icp_matrices])
    post_orientations = np.array([matrix[:2, 0] for matrix in post_icp_matrices])

    # Plotting the pre-ICP translations and orientations
    plt.figure(figsize=(10, 8))
    plt.scatter(pre_translations[:, 0], pre_translations[:, 1], marker='o', color='b', label='Pre-ICP', s=3)
    for i in range(len(pre_translations)):
        if i % 30 == 0:
            plt.arrow(pre_translations[i, 0], pre_translations[i, 1], pre_orientations[i, 0], pre_orientations[i, 1],
                      head_width=0.4, head_length=1.5, fc='b', ec='b')

    # Plotting the post-ICP translations and orientations
    plt.scatter(post_translations[:, 0], post_translations[:, 1], marker='x', color='r', label='Post-ICP', s=3)
    for i in range(len(post_translations)):
        if i % 30 == 0:
            plt.arrow(post_translations[i, 0], post_translations[i, 1], post_orientations[i, 0],
                      post_orientations[i, 1],
                      head_width=0.4, head_length=1.5, fc='r', ec='r')

    plt.title('Point Cloud Translations and Orientations in Local Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()



def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])
def filter_pointcloud_z(pcd):
    # points = np.asarray(pcd.points)
    # mask = points[:, 2] <= 400000
    # filtered_points = points[mask]
    # pcd.points = o3d.utility.Vector3dVector(filtered_points)
    return pcd

def filter_pointcloud_x(pcd, x_min=-120000):
    # points = np.asarray(pcd.points)
    # mask = points[:, 0] >= x_min
    # filtered_points = points[mask]
    # pcd.points = o3d.utility.Vector3dVector(filtered_points)
    return pcd

def plot_euler_angles(euler_angles_list):
    euler_angles_array = np.array(euler_angles_list)
    time_steps = np.arange(len(euler_angles_array))

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle('Andamento degli Angoli di Eulero')

    axs[0].plot(time_steps, euler_angles_array[:, 0], label='Roll')
    axs[0].set_ylabel('Roll (rad)')
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(time_steps, euler_angles_array[:, 1], label='Pitch')
    axs[1].set_ylabel('Pitch (rad)')
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(time_steps, euler_angles_array[:, 2], label='Yaw')
    axs[2].set_xlabel('Time step')
    axs[2].set_ylabel('Yaw (rad)')
    axs[2].grid(True)
    axs[2].legend()

    plt.show()

def convert_to_meters(pcd):
    points = np.asarray(pcd.points) / 1000.0  # Convert mm to meters
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def convert_to_mm(pcd):
    points = np.asarray(pcd.points) * 1000.0  # Convert mm to meters
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def remove_isolated_points(pcd, nb_neighbors=10, radius=500):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned


def create_plane_pointcloud(x_shift=0.0):
    # Creare una griglia di punti su un piano
    xx, yy = np.meshgrid(np.arange(-1, 1, 0.1), np.arange(-1, 1, 0.1))
    zz = np.zeros_like(xx)
    points = np.vstack((xx.flatten(), yy.flatten(), zz.flatten())).T

    # Traslare il piano lungo l'asse X
    points[:, 0] += x_shift

    # Creare una point cloud da questi punti
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def plot_timestamps_comparison(gnss_timestamps, pointcloud_timestamps):
    plt.figure(figsize=(15, 5))
    plt.scatter(gnss_timestamps, np.ones_like(gnss_timestamps), marker='o', color='b', label='GNSS Timestamps')
    plt.scatter(pointcloud_timestamps, np.zeros_like(pointcloud_timestamps), marker='x', color='r',
                label='Point Cloud Timestamps')
    plt.yticks([0, 1], ['Point Cloud', 'GNSS'])
    plt.xlabel('Timestamp')
    plt.title('Comparison of GNSS and Point Cloud Timestamps')
    plt.legend()
    plt.grid(True)
    plt.show()
# Main Execution



def main():
    gnss_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/gnss_data.json'
    pointcloud_dir = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak/pc'

    gnss_array, local_coordinates, reference_latitude, reference_longitude, scale_factor = plot_gnss_trajectory(gnss_file)






    interp_lat, interp_lon, interp_alt = interpolate_gnss_data(gnss_array, local_coordinates)
    pointcloud_files, transformation_matrices = associate_pointclouds_with_gnss(pointcloud_dir, interp_lat, interp_lon,
                                                                                interp_alt, reference_latitude,
                                                                                reference_longitude, scale_factor)


    # Extract timestamps for plotting
    gnss_timestamps = gnss_array[:, 0]
    pointcloud_timestamps = [float(re.search(r'(\d+\.\d+)', f).group(1)) for f in pointcloud_files]

    # Plot timestamps comparison
    #plot_timestamps_comparison(gnss_timestamps, pointcloud_timestamps)

    transformation_matrices_original = transformation_matrices.copy()


    # Array per memorizzare le point cloud filtrate
    pointclouds_filtered_z = []
    pointclouds_filtered_zx = []


    # Ciclo per processare tutte le pointcloud e applicare i filtri
    for idx in range(len(pointcloud_files)):
        pcd_raw = o3d.io.read_point_cloud(os.path.join(pointcloud_dir, pointcloud_files[idx]))
        pcd = remove_isolated_points(pcd_raw)
        pcd_m = convert_to_meters(pcd)

        # Filtra la point cloud in z background
        pcd_filtered_z = filter_pointcloud_z(pcd_m)
        pointclouds_filtered_z.append(pcd_filtered_z)

        # Filtra la point cloud in z e poi in x terreno
        pcd_filtered_zx = filter_pointcloud_x(pcd_filtered_z)
        pointclouds_filtered_zx.append(pcd_filtered_zx)


    processed_pointcloud_z = []
    processed_pointcloud_zx_filtered = []
    processed_pointcloud_z_specific = []
    target = None
    euler_angles_list = []

    timestamp_min = 1717150093.9398425
    timestamp_max = 1717150096.7073352


    for idx in range(1, len(pointcloud_files)):





        # rotazione della pointclpud precedente
        initial_transformation = np.eye(4)
        initial_transformation[:3, :3] = transformation_matrices[idx - 1][:3, :3]  # Set initial rotation from previous

        # POSIZIONO LA POINTCLOUD AFFIANCO ALLA PRIMA
        initial_transformation[:3, 3] = transformation_matrices[idx][:3, 3]  # Set initial translation from GNSS
        # print("INITIAL:", initial_transformation)

        # prendo le pointcloud senza baground e senza terreno e dal loro SDR relativo le trasformo nella posizione del GNSS e nell orientazione della PC precedente

        source_filtered_z = pointclouds_filtered_z[idx]
        source_filtered_temp = pointclouds_filtered_zx[idx]  # TERRENO x icp

        source_filtered_z.transform(initial_transformation)
        source_filtered_temp.transform(initial_transformation)

        if idx == 1:

            target_filtered_z = pointclouds_filtered_z[0]
            # POSIZIONA LA PRIMA SUL TRACCIATO: no background
            target_filtered_z.transform(transformation_matrices[0])
            processed_pointcloud_z.append(target_filtered_z)

            target_filtered_temp = pointclouds_filtered_zx[0]
            # POSIZIONA LA PRIMA SUL TRACCIATO: no terreno
            target_filtered_temp.transform(transformation_matrices[0])
            processed_pointcloud_zx_filtered.append(target_filtered_temp)

        else:
            target_filtered_z = processed_pointcloud_z[idx - 1]
            target_filtered_temp = processed_pointcloud_zx_filtered[idx - 1]  # TERRENO x icp



        timestamp = float(re.search(r'\d+\.\d+', pointcloud_files[idx]).group())
        if timestamp > timestamp_min and timestamp < timestamp_max:
            print(idx)
            #print("icp:",idx)



            # if idx > 1759 and idx < 2000:
            if 0:

                source_filtered_temp.paint_uniform_color([1, 0.706, 0])  # yellow
                target_filtered_temp.paint_uniform_color([0, 0.651, 0.929])  # blue
                o3d.visualization.draw_geometries([source_filtered_temp, target_filtered_temp], window_name=f'ICP Alignment {idx}')





            #ADESSO ACNHE LA SOURCE é PIAZZATA SUL TRACCIATO

            original_source = source_filtered_temp



            # Perform manual ICP to refine both rotation and translation
            icp_result = icp_manual(source_filtered_temp, target_filtered_temp,
                                    np.eye(4))  # Start ICP with identity since source is pre-transformed

            if np.isnan(icp_result[:3, 3]).any():
                print(f"ICP result for pointcloud {idx} contains NaN values in translations, skipping this iteration.")
                print(source_filtered_temp,target_filtered_temp)

                source_filtered_temp.paint_uniform_color([1, 0.706, 0])  # yellow
                target_filtered_temp.paint_uniform_color([0, 0.651, 0.929])  # blue
                o3d.visualization.draw_geometries([source_filtered_temp, target_filtered_temp], window_name=f'ICP Alignment {idx}')

                merged_pointcloud = o3d.geometry.PointCloud()
                for pcd in processed_pointcloud_z:
                    merged_pointcloud += pcd

                # Visualize the merged point cloud
                o3d.visualization.draw_geometries([merged_pointcloud], window_name='Merged Point Cloud')

                break  # Salta l'iterazione se le traslazioni contengono NaN

            # sommo la rotazione trovata dall icp alla rotazione precedente
            final_transformation = np.eye(4)
            final_transformation[:3, :3] = icp_result[:3, :3] @ initial_transformation[:3, :3]  # Combine rotations
            # sommo la traslazione dell icp alla precedente
            final_transformation[:3, 3] = initial_transformation[:3, 3] + icp_result[:3, 3]  # Combine translations


            #calcolo gli angoli euleriani
            euler_angles = rotation_matrix_to_euler_angles(final_transformation[:3, :3])
            euler_angles_list.append(euler_angles)
            # Convert to degrees and print
            euler_angles_deg = np.degrees(euler_angles)
            #print(f"DEG pointcloud {idx}: Roll: {euler_angles_deg[0]:.2f}, Pitch: {euler_angles_deg[1]:.2f}, Yaw: {euler_angles_deg[2]:.2f}")
            #print(source_filtered_temp)

            # Update the transformation matrix with the new rotation and translation
            #AGGIORNO IL SDR corrente con l incremento dell ICP
            transformation_matrices[idx] = final_transformation
            #print("ICP:", icp_result)

            # APPLICOO L incremento dell icp sulla poincloud senza BG
            source_filtered_z.transform(icp_result)  # Apply only the ICP result to the already rotated point cloud
            #PER IL total filter
            source_filtered_temp.transform(icp_result)  # Apply only the ICP result to the already rotated point cloud

            processed_pointcloud_z_specific.append(source_filtered_z)

            if 0:
                print(original_source,source_filtered_z)

                source_filtered_temp.paint_uniform_color([0, 0.706, 0.5])  # yellow
                target_filtered_temp.paint_uniform_color([0.5, 0.0651, 0.929])  # blue
                o3d.visualization.draw_geometries([original_source, source_filtered_z], window_name=f'ICP Alignment {idx}')


            # merged_pointcloud = o3d.geometry.PointCloud()
            # if len(processed_pointcloud_z_specific) > 1:
            #     merged_pointcloud += processed_pointcloud_z_specific[-2]
            #     merged_pointcloud += processed_pointcloud_z_specific[-1]
            #
            # # Visualizzare la point cloud unita
            # o3d.visualization.draw_geometries([merged_pointcloud], window_name='Merged Point Cloud')

        processed_pointcloud_z.append(source_filtered_z)
        processed_pointcloud_zx_filtered.append(source_filtered_temp)


        #FINE



    plot_pointcloud_translations(transformation_matrices_original,transformation_matrices)
    plot_euler_angles(euler_angles_list)