import custom_slam_doc as CUSTOM_SLAM
from pyk4a import PyK4APlayback, Config, PyK4A
from pyk4a import PyK4APlayback, CalibrationType
import open3d as o3d
import numpy as np
import os
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pandas as pd
import cv2
import matplotlib.pyplot as plt
import ctypes
import pyk4a
import matrix_utilities as MATRIX_UTILS
import PREPROC_double_KA as KA_PREPROC
import re
import sys
from decorator import *
import localization_proc as LOCALIZE


def plot_trajectory_3d(timestamp_dict):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Ordina i timestamp
    sorted_timestamps = sorted(timestamp_dict.keys())

    # Inizializza la posizione iniziale (matrice identità)
    current_position = np.eye(4)

    # Liste per memorizzare le coordinate x, y, z
    x = [current_position[0, 3]]
    y = [current_position[1, 3]]
    z = [current_position[2, 3]]

    # Applica le trasformazioni in ordine di timestamp
    for ts in sorted_timestamps:
        transform = timestamp_dict[ts]
        current_position = np.dot(current_position, transform)
        x.append(current_position[0, 3])
        y.append(current_position[1, 3])
        z.append(current_position[2, 3])

    # Plotta la traiettoria 3D
    ax.plot(x, y, z, marker='o', linewidth=0.2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory 3D Plot')

    # Assicura che gli assi abbiano la stessa scala
    ax.set_aspect('auto')
    max_range = np.array([max(x)-min(x), max(y)-min(y), max(z)-min(z)]).max()
    mid_x = (max(x) + min(x)) * 0.5
    mid_y = (max(y) + min(y)) * 0.5
    mid_z = (max(z) + min(z)) * 0.5
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

    plt.show()

def downsample_point_cloud(point_cloud, voxel_size):
    """
    Downsample a point cloud using a voxel grid filter and remove duplicated points.

    Parameters:
        point_cloud (o3d.geometry.PointCloud): The input point cloud.
        voxel_size (float): The voxel size for downsampling.

    Returns:
        o3d.geometry.PointCloud: The downsampled point cloud.
    """
    # Downsample the point cloud using voxel grid filter
    downsampled_pc = point_cloud.voxel_down_sample(voxel_size)

    # Remove duplicate points
    downsampled_pc = downsampled_pc.remove_duplicated_points()

    return downsampled_pc


def get_sorted_filenames_by_timestamp(directory):
    filenames = os.listdir(directory)
    files_with_timestamps = []
    for filename in filenames:
        if not filename.endswith('.ply'):
            continue
        timestamp = os.path.splitext(filename)[0].split('_')[-1]
        files_with_timestamps.append((filename, timestamp))
    files_with_timestamps.sort(key=lambda x: x[1])
    sorted_filenames = [x[0] for x in files_with_timestamps]
    return sorted_filenames

def visualize_pc(pc, title):
    point_size = 2
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)

    # Aggiungi le geometrie una alla volta
    vis.add_geometry(pc)


    opt = vis.get_render_option()
    # Imposta il colore di sfondo (colore RGB normalizzato)
    opt.background_color = np.array([0.1, 0.1, 0.1])  # Colore nero
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()



def icp_open3d_coupling(source, target,LIMIT_TRASFORMATION  = 0, max_iterations=250, threshold=0.05):
    """
    Esegue l'ICP tra due nuvole di punti e controlla che la trasformazione risultante sia entro i limiti accettabili.

    Args:
        source (o3d.geometry.PointCloud): La nuvola di punti sorgente.
        target (o3d.geometry.PointCloud): La nuvola di punti target.
        max_iterations (int): Il numero massimo di iterazioni per l'ICP.
        threshold (float): La soglia di distanza per l'ICP.
        rotation_threshold (float): La soglia per la parte rotazionale della trasformazione.
        translation_threshold (float): La soglia per la parte traslazionale della trasformazione.

    Returns:
        np.ndarray: La matrice di trasformazione risultante se valida, altrimenti la matrice di identità.
    """
    translation_threshold = 5
    rotation_threshold = 0.1

    initial_transformation = np.eye(4)
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )

    transformation = icp_result.transformation
    rotation = transformation[:3, :3]
    translation = transformation[:3, 3]
    def is_valid_rotation_matrix(R):
        should_be_identity = np.dot(R.T, R)
        I = np.identity(3, dtype=R.dtype)
        return np.allclose(should_be_identity, I) and np.isclose(np.linalg.det(R), 1.0)

    if not is_valid_rotation_matrix(rotation):
        print("______INVALID_ROT_ERROR.")
        return np.eye(4)

    # Clamping del valore per arccos
    cos_angle = (np.trace(rotation) - 1) / 2
    cos_angle = np.clip(cos_angle, -1, 1)

    angle = np.arccos(cos_angle)
    translation_magnitude = np.linalg.norm(translation)
    #print("ICP RESULTS: TRASAL:", translation_magnitude, " ROT:",angle)

    if LIMIT_TRASFORMATION:
        # Controlla la parte rotazionale

        if angle > rotation_threshold:
            print("ICP IVALID ROTATION.",angle)
            return np.eye(4)

        # Controlla la parte traslazionale

        if translation_magnitude > translation_threshold:
            print("________________ICP IVALID TRASLATION.",translation_magnitude)
            return np.eye(4)

    return transformation


def get_pointcloud_timestamps(directory):
    """
    Get a set of timestamps from the point cloud filenames in the given directory.
    Assumes filenames are in the format 'pointcloud_<timestamp>.ply'.
    """
    timestamps = set()
    for filename in os.listdir(directory):
        if filename.startswith("pointcloud_") and filename.endswith(".ply"):
            timestamp = filename[len("pointcloud_"):-len(".ply")]
            timestamps.add(int(timestamp))
    return timestamps


def find_closest_timestamp(timestamp, timestamp_set, max_difference=5000):
    """
    Find the closest timestamp in the timestamp_set to the given timestamp.
    Only consider differences up to max_difference milliseconds.
    """
    closest_timestamp = None
    min_difference = max_difference
    for ts in timestamp_set:
        difference = abs(timestamp - ts)
        if difference < min_difference:
            min_difference = difference
            closest_timestamp = ts
    return closest_timestamp, min_difference


def plot_timestamps(timestamps1, timestamps2, unmatched1, unmatched2):
    """
    Plot timestamps from two directories and highlight unmatched timestamps.
    """
    plt.figure(figsize=(12, 6))

    # Plot timestamps
    plt.scatter(timestamps1, [1] * len(timestamps1), label='Dir1', color='blue')
    plt.scatter(timestamps2, [2] * len(timestamps2), label='Dir2', color='green')

    # Highlight unmatched timestamps
    plt.scatter(unmatched1, [1] * len(unmatched1), color='red', label='Unmatched Dir1')
    plt.scatter(unmatched2, [2] * len(unmatched2), color='orange', label='Unmatched Dir2')

    plt.xlabel('Timestamps')
    plt.yticks([1, 2], ['Dir1', 'Dir2'])
    plt.title('Timestamps Comparison')
    plt.legend()
    plt.show()
def compare_directories(dir1, dir2):
    """
    Compare the point cloud files in two directories based on their timestamps.
    """
    timestamps1 = get_pointcloud_timestamps(dir1)
    timestamps2 = get_pointcloud_timestamps(dir2)

    only_in_dir1 = timestamps1 - timestamps2
    only_in_dir2 = timestamps2 - timestamps1
    in_both = timestamps1 & timestamps2

    print(f"Total timestamps in {dir1}: {len(timestamps1)}")
    print(f"Total timestamps in {dir2}: {len(timestamps2)}")
    print(f"Timestamps in both directories: {len(in_both)}")

    paired_timestamps = list(in_both)

    close_matches_dir1 = {}
    close_matches_dir2 = {}

    for ts in sorted(only_in_dir1):
        closest_ts, difference = find_closest_timestamp(ts, timestamps2)
        if closest_ts is not None:
            close_matches_dir1[ts] = (closest_ts, difference)

    for ts in sorted(only_in_dir2):
        closest_ts, difference = find_closest_timestamp(ts, timestamps1)
        if closest_ts is not None:
            close_matches_dir2[ts] = (closest_ts, difference)

    print(f"\nClose matches for timestamps only in {dir1}:")
    for ts, (closest_ts, diff) in close_matches_dir1.items():
        print(f"{ts} -> {closest_ts} (difference: {diff} ms)")

    print(f"\nClose matches for timestamps only in {dir2}:")
    for ts, (closest_ts, diff) in close_matches_dir2.items():
        print(f"{ts} -> {closest_ts} (difference: {diff} ms)")

    unmatched_dir1 = sorted(only_in_dir1 - set(close_matches_dir1.keys()))
    unmatched_dir2 = sorted(only_in_dir2 - set(close_matches_dir2.keys()))

    print(f"\nUnmatched timestamps only in {dir1}: {unmatched_dir1}")
    print(f"Unmatched timestamps only in {dir2}: {unmatched_dir2}")

    print(f"\nTotal close matches for {dir1}: {len(close_matches_dir1)}")
    print(f"Total close matches for {dir2}: {len(close_matches_dir2)}")
    print(f"Total unmatched in {dir1}: {len(unmatched_dir1)}")
    print(f"Total unmatched in {dir2}: {len(unmatched_dir2)}")

    # Plotting the timestamps
    plot_timestamps(sorted(timestamps1), sorted(timestamps2), unmatched_dir1, unmatched_dir2)


def extract_pointcloud_and_sort(output_folder_pc_1):

    pointcloud_files = [f for f in os.listdir(output_folder_pc_1) if f.endswith('.ply')]
    pointcloud_timestamps = []
    for file in pointcloud_files:
        match = re.search(r'(\d+\.\d+)', file)
        if match:
            timestamp = float(match.group(1))
            pointcloud_timestamps.append(timestamp)

    # Sort files and timestamps together
    pointcloud_files, pointcloud_timestamps = zip(*sorted(zip(pointcloud_files, pointcloud_timestamps)))
    pointcloud_files_sorted = sorted(pointcloud_files, key=get_timestamp)
    start = 2297
    timestamp_sorted = []
    pointclouds = []
    for idx, file_name in enumerate(pointcloud_files_sorted):
        timestamp_str = file_name.split('_')[1].split('.ply')[0]
        timestamp = float(timestamp_str)
        timestamp_sorted.append(timestamp)

        pcd_raw = o3d.io.read_point_cloud(
            os.path.join(output_folder_pc_1, file_name))  # Usa file_name invece di pointcloud_files[idx]
        pcd_m = convert_to_meters(pcd_raw)
        pointclouds.append(pcd_m)

    return pointclouds,timestamp_sorted


def transform_all_lower_pc_plus_icp_registration(pointclouds_1, pointclouds_2, initial_transform,
                                                 coupled_saving_folder):
    """
    Trasforma e registra due liste di nuvole di punti con timestamp corrispondenti.
    """
    pointclouds_1 = [os.path.join(pointclouds_1, f) for f in get_sorted_filenames_by_timestamp(pointclouds_1)]
    pointclouds_2 = [os.path.join(pointclouds_2, f) for f in get_sorted_filenames_by_timestamp(pointclouds_2)]

    for pc1_path in pointclouds_1:
        timestamp1 = os.path.splitext(os.path.basename(pc1_path))[0].split('_')[-1]
        matching_pc2_path = None
        for pc2_path in pointclouds_2:
            timestamp2 = os.path.splitext(os.path.basename(pc2_path))[0].split('_')[-1]
            if timestamp1 == timestamp2:
                matching_pc2_path = pc2_path
                break

        if matching_pc2_path is None:
            source_pc = o3d.io.read_point_cloud(pc1_path)
            output_path = os.path.join(coupled_saving_folder, f"pointcloud_{timestamp1}.ply")
            if not os.path.exists(output_path):
                o3d.io.write_point_cloud(output_path, source_pc)
            else:
                print("skipped")
            print("NONE MATCH")
        else:
            print("COUPLING: ",timestamp1)
            source_pc = o3d.io.read_point_cloud(pc1_path)
            target_pc = o3d.io.read_point_cloud(matching_pc2_path)

            target_pc.transform(initial_transform)
            final_transform = icp_open3d_coupling(source_pc, target_pc, 1)
            target_pc.transform(final_transform)

            combined_pc = source_pc + target_pc
            #visualize_pc(combined_pc)

            output_path = os.path.join(coupled_saving_folder, f"pointcloud_{timestamp1}.ply")
            if not os.path.exists(output_path):
                o3d.io.write_point_cloud(output_path, combined_pc)
            else:
                print("skipped")


def convert_to_meters(pcd):
    points = np.asarray(pcd.points) / 1000.0  # Convert mm to meters
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def remove_isolated_points(pcd, nb_neighbors=10, radius=500.0):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned

@timeit
def hierarchy_slam_icp(input_all_pointcloud,timestamp_sorted):

    # devo saltare la zero che è solo target
    # prendo la sua trasformata trovata tra le sotto pc e la applico alla coppia dopo
    # current_source.trasform(last_epoch_trasformation[i-1])
    # NDR la trasformazione [i-esima] l ho gia usata l epoca prima per traslare la PC
    # tecnicamente cosi ho PC 0,1,2,3  0-1 2-3 gia fuse, applicando questa nuova tresformazione in pratica sto sovrapponendo pc2 a pc1 cosi avremo in serie 0 1+2 e 3
    # la trasformazione nuova mi farà avanzare 2-3 alla fine di 1,
    # ogni iterazione individui la trasformazione 1-2 che però potrebbe crescere con le iterazioni (dipende na quante epoche hanno costruito 1)
    # se volessi avvicinarle ancora dovrei sommare la trasformazione 2-3 anche, (da rivedere perche potrebbe essere troppo)
    # fuse PC[0],PC[1]
    # fuse PC[i],PC[i+1]
    # trasformation_estimated = icp
    # trasfoirmation_current.append(trasformation_estimated)
    # source-trasform to match target ->(trasformation_estimated)
    # fuse target and source transformed
    # new_matrices.append(fusion)

    # al primo giro prendo PC grezze
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0

    last_epoch_trasformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]
    x_y = []
    timestamp_dict = {}

    while len(epoch_start_pointclouds) > 1:
        start_time = time.time()
        epoch += 1
        i = 1

        new_halfed_pointcloud = []
        trasformation_current = []

        print(f"START Epoch {epoch}, PCs:{len(epoch_start_pointclouds)}")


        while i < len(epoch_start_pointclouds):


            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(len(last_epoch_trasformation))
                print(len(epoch_start_pointclouds))

            #TOGLILO
            if i > 0:

                #print(i, end=" ")

                initial_trasform_from_last_epoch_1_couple = (last_epoch_trasformation[i-1])
                initial_trasform_from_last_epoch_2_couple = (last_epoch_trasformation[i])

                # CONSIDERO I PRIOR DI ENTRAMBE LE SOURCE PASSATE, PERCHE SE NO PIU AUMENTANO LE EPOCE PIU AUMENTA IL LAG TRA LE POINTCLOUS.
                # QUINDI PRENDO IL PRIOR TRASFORMATION DA ENTRAMBE LE PC CHE HANNO COSTRUITO LA CORRENTE.



                #Salvolo entrambe però perche l iterazione dopo saranno lunghe il doppio e mi servve la lunghezza di una piu l altra per traslare
                prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)
                #prior_trasformation_composed = initial_trasform_from_last_epoch_2_couple

                source_raw = epoch_start_pointclouds[i]

                current_source = o3d.geometry.PointCloud(source_raw)
                # devo aggiungere anche la trasformata delle iterazioni prima BASTA CHE LE LAST EPOCH LA COMPRENDANO

                current_source.transform(initial_trasform_from_last_epoch_1_couple)

                target =  epoch_start_pointclouds[i-1]


                VOXEL_VOLUME = 0.03
                current_source = downsample_point_cloud(current_source, VOXEL_VOLUME)
                target = downsample_point_cloud(target, VOXEL_VOLUME)



                SHOW_ICP_PROCESS = 0
                if SHOW_ICP_PROCESS:
                    if len(current_source.points) < 600 or len(target.points) < 600:
                        target = remove_isolated_points(target, nb_neighbors=12, radius=0.7)
                        current_source = remove_isolated_points(current_source, nb_neighbors=12, radius=0.8)

                    yellow = np.array([1, 1, 0])
                    red =  np.array([1, 0, 0])
                    green = np.array([0, 1, 0])
                    blue = np.array([0, 0, 1])

                    # print("source",len(current_source.points))
                    # print("target", len(target.points))
                    if len(current_source.points) < 600 or len(target.points) < 600:
                        target = remove_isolated_points(target, nb_neighbors=12, radius=0.7)
                        current_source = remove_isolated_points(current_source, nb_neighbors=12, radius=0.8)

                    current_source.colors = o3d.utility.Vector3dVector(np.tile(yellow, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(red, (len(target.points), 1)))
                    pre_icp = target + current_source
                    visualize_pc(pre_icp, "pre icp")

                    # Imposta tutti i punti al colore bianco (RGB: [1, 1, 1])
                    white = np.array([1, 1, 1])
                    current_source.colors = o3d.utility.Vector3dVector(np.tile(green, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(blue, (len(target.points), 1)))






                updated_trasform_icp_result = CUSTOM_SLAM.icp_open3d(current_source, target, 0.05, 200)
                #Total trasformation : current, plus the prrevious frame trasf:
                x_y.append(updated_trasform_icp_result[0, -3:])
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)


                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                if len(epoch_start_pointclouds) == 2:
                    print(updated_trasform_icp_result)

                    # pre_icp  = target + current_source
                    # visualize_pc(pre_icp, "pre icp")


                merged_pcd = target + trasformed_icp_source





                if SHOW_ICP_PROCESS:
                    visualize_pc(merged_pcd, "merged")
                    current_source.colors = o3d.utility.Vector3dVector(np.tile(white, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(white, (len(target.points), 1)))



                # Controllo se esiste una point cloud precedente e successiva
                # Controllo se esiste una point cloud precedente e successiva

                PREVIOUS_ADDICTION = 0
                if PREVIOUS_ADDICTION:
                    #all inizio non ha senso usarlo perche la percentuale di sovrapposizione è alta
                    if epoch > 4:
                        if i - 2 >= 0:
                            previous_pc = epoch_start_pointclouds[i - 2]
                            initial_transform_previous_1 = last_epoch_trasformation[i - 2]
                            initial_transform_previous_2 = last_epoch_trasformation[i - 1]
                            prior_transform_composed_previous = np.dot(initial_transform_previous_1,
                                                                       initial_transform_previous_2)

                            #COME AGGIORNATO BASTA UNA SOLA TRASFORMAZIONE
                            # Trasformare la merged_pcd
                            merged_pcd.transform(initial_transform_previous_1)
                            # Eseguire ICP per allineare merged_pcd con previous_pc
                            previous_pc_transformed = o3d.geometry.PointCloud(previous_pc)
                            updated_transform_icp_result_previous = CUSTOM_SLAM.icp_open3d(merged_pcd, previous_pc_transformed)

                            # Correggere merged_pcd con la trasformazione ICP trovata
                            merged_pcd.transform(updated_transform_icp_result_previous)
                            merged_pcd += previous_pc_transformed

                NEXT_ADDITION = 0
                if NEXT_ADDITION:


                    if i + 1 < len(epoch_start_pointclouds):
                        next_pc = epoch_start_pointclouds[i + 1]


                        #COME AGGIORNATO INVECE CHE 2 NE BASTA UNA INVECE CHE 6 NE BASTANO 3

                        # Coppia di trasformazioni per unire la previous
                        transform_previous_1 = last_epoch_trasformation[i - 2]
                        transform_previous_2 = last_epoch_trasformation[i - 1]
                        transform_composed_previous = np.dot(transform_previous_1, transform_previous_2)

                        # Coppia di trasformazioni per unire la current
                        transform_current_1 = last_epoch_trasformation[i]
                        transform_current_2 = last_epoch_trasformation[i + 1]
                        transform_composed_current = np.dot(transform_current_1, transform_current_2)

                        # Coppia di trasformazioni per unire la next (per la prossima iterazione)
                        transform_next_1 = last_epoch_trasformation[i]
                        transform_next_2 = last_epoch_trasformation[i + 1]
                        transform_composed_next = np.dot(transform_next_1, transform_next_2)

                        # Combinare tutte le trasformazioni
                        combined_transform = np.dot(transform_previous_1, transform_current_1)
                        combined_transform = np.dot(combined_transform, transform_next_1)

                        # Trasformare la merged_pcd con le trasformazioni combinate
                        next_pc.transform(combined_transform)

                        # Eseguire ICP per allineare merged_pcd con next_pc
                        next_pc_transformed = o3d.geometry.PointCloud(next_pc)
                        transform_icp_result_next = CUSTOM_SLAM.icp_open3d(merged_pcd, next_pc_transformed)

                        # Correggere merged_pcd con la trasformazione ICP trovata
                        next_pc_transformed.transform(transform_icp_result_next)
                        merged_pcd += next_pc_transformed


                merged_pcd = remove_isolated_points(merged_pcd, nb_neighbors=12, radius=0.4)
                trasformation_current.append(total_trasformation_prior_plus_icp)
                new_halfed_pointcloud.append(merged_pcd)


                # i = 97, len = 99,
                # quando i = 99 rompe e quindi perderò la pointclou i[98], che sarabbe il target di i[99] che non arriveà
                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:  # Vogliamo eliminare
                # SKip the odd last matrixprint
                   # 100/101 i  = 100

                    print("DISPARI")
                    print(f"START Epoch {epoch},i = {i}, PCs AVIABLE:{len(epoch_start_pointclouds)} PCS comp:{len(new_halfed_pointcloud)} trasform:{len(trasformation_current)}")
                    print("add last PC with its trasformation")
                    # TO DO TO INCLUDE LAST
                    last_pc = epoch_start_pointclouds[i+1] #98
                    last_trasaform = (last_epoch_trasformation[i+1]) #98

                    trasformation_current.append(last_trasaform)
                    new_halfed_pointcloud.append(last_pc)

                timestamp_index = i // 2 if epoch == 1 else len(timestamp_sorted) // (2 ** (epoch - 1)) + i // 2
                timestamp_dict[timestamp_sorted[timestamp_index]] = updated_trasform_icp_result

            i += 2


        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_trasformation = trasformation_current
        end_time = time.time()
        elapsed_time = end_time - start_time
        elapsed_time_ms = round(elapsed_time, 3)
        print(f"Computed Epoch {epoch} in {elapsed_time_ms} s, PCs and T created:{len(new_halfed_pointcloud)}")


    # Estraiamo i valori di x e y
    x_values = [item[0] for item in x_y]
    y_values = [item[1] for item in x_y]
    z_values = [item[2] for item in x_y]




    plot_trajectory_3d(timestamp_dict)


    return new_halfed_pointcloud[0]

def get_timestamp(file_name):
    timestamp_str = file_name.split('_')[1].split('.ply')[0]

    return float(timestamp_str)



if 1:

    input_file_mkv_1 ="/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/mastermkv/20240716_113102_master.mkv"
    input_file_mkv_2= "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/slavemkv/20240716_113058_subordinate.mkv"
    output_folder_pc_1 = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/output_double/raw_master"
    output_folder_pc_2 = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/output_double/raw_slave"
    start_index_1 = 700
    start_index_2 = start_index_1
    end_index_1 = 5220
    end_index_2 = end_index_1
    timestamp_conversion_file_1 = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/mastermkv/20240716_113102_data.csv"
    timestamp_conversion_file_2 = timestamp_conversion_file_1
    coupled_saving_folder = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/output_double/coupled_saved"


    LOCALIZE_ACQUISITION = 1
    if LOCALIZE_ACQUISITION:
        imu_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/localization_data/imu_data.json'
        gnss_data = LOCALIZE.load_json('/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/localization_data/gnss_data.json')
        imu_data = LOCALIZE.load_json(imu_file)

        # Interpolate GNSS data for IMU timestamps
        interpolated_gnss_data = LOCALIZE.interpolate_gnss_for_imu(gnss_data, imu_data)
        LOCALIZE.plot_gnss_data(interpolated_gnss_data)

        timestamps, trajectory, linear_accelerations, global_accelerations, rotations = LOCALIZE.process_imu_data(imu_file)

        # Plot the translations


        # Plot the linear accelerations
        # LOCALIZE.plot_accelerations(timestamps, linear_accelerations)
        # LOCALIZE.plot_accelerations(timestamps, global_accelerations)
        # LOCALIZE.plot_accelerations_and_rotations(timestamps, linear_accelerations, rotations)
        # LOCALIZE.plot_accelerations_and_rotations(timestamps, global_accelerations,rotations)

        # # Plot the rotations
        # LOCALIZE.plot_rotations(timestamps, rotations)

        # Plot the 3D trajectory
        LOCALIZE.plot_trajectory_3d_imuu(timestamps, trajectory)
        sys.exit()


    SHOW_RGB_MKV = 0
    if SHOW_RGB_MKV:

        KA_PREPROC.show_rgb_video_mkv(input_file_mkv_2)

        sys.exit()





    EXTRACT_RAW_PCS = 0
    if EXTRACT_RAW_PCS:

        KA_PREPROC.process_mkv(input_file_mkv_1,output_folder_pc_1,start_index_1,timestamp_conversion_file_1,end_index_1)
        KA_PREPROC.process_mkv(input_file_mkv_2,output_folder_pc_2,start_index_2,timestamp_conversion_file_2, end_index_2)

    #compare_directories(output_folder_pc_1,output_folder_pc_2)

    COUPLE_RAW_PC = 0
    if COUPLE_RAW_PC:

        #__________________________COUPLING________________________#
        height_displacement = 853 #[mm]
        lateral_displacement = 6

        initial_trasform_fixed_high_camera = np.eye(4)

        # Aggiungi la traslazione su x di 853 e su z di 6
        initial_trasform_fixed_high_camera[0, 3] = 853  # Traslazione su x
        initial_trasform_fixed_high_camera[2, 3] = 6    # Traslazione su z

        transform_all_lower_pc_plus_icp_registration(output_folder_pc_1, output_folder_pc_2, initial_trasform_fixed_high_camera,coupled_saving_folder)
        # take this list of pointclpouds and save with: pointcloud_<timestamp> in a given folder
        # save here : coupled_saving_folder
        sys.exit()
    # define the initial trasformation between the upper and lower kinect
    # zippa le coppie di pointcloud basandoti sul loro timestamp
    # and then peorform ICP to fuse upper and lower PC
    # salvale in una terzo folder da cui poi le ripescherai

    pointcloud_files = [f for f in os.listdir(coupled_saving_folder) if f.endswith('.ply')]

    pointcloud_timestamps = []
    for file in pointcloud_files:
        match = re.search(r'(\d+)', file)
        if match:
            timestamp = int(match.group(1))
            pointcloud_timestamps.append(timestamp)

    if pointcloud_timestamps:
        # Sort files and timestamps together
        pointcloud_files, pointcloud_timestamps = zip(*sorted(zip(pointcloud_files, pointcloud_timestamps)))
        pointcloud_files_sorted = list(pointcloud_files)
    else:
        print("ERROR NO TIMESTAMP")
        pointcloud_files_sorted = []

    print("total PCs", len(pointcloud_files))
    starts = 200
    ends = 2000

    print("analizing: S:", starts, " E:", ends, " TOT:", ends-starts)
    timestamp_sorted = []
    pointclouds = []
    for idx, file_name in enumerate(pointcloud_files_sorted):


        timestamp_str = file_name.split('_')[1].split('.ply')[0]
        timestamp = float(timestamp_str)

        if idx > starts and idx < ends:
            pcd_raw = o3d.io.read_point_cloud(
                os.path.join(coupled_saving_folder, file_name))  # Usa file_name invece di pointcloud_files[idx]
            pcd_raw = convert_to_meters(pcd_raw)

            pointclouds.append(pcd_raw)
            timestamp_sorted.append(timestamp)



    fused = hierarchy_slam_icp(pointclouds,timestamp_sorted)
    filename = "output_double/fused.ply"
    o3d.io.write_point_cloud(filename, fused)
    print(f"PointCloud salvata come {filename}")


    visualize_pc(fused,"end")

    sys.exit()
