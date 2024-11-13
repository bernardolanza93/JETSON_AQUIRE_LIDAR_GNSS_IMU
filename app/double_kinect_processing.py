import custom_slam_doc as CUSTOM_SLAM
from pyk4a import PyK4APlayback, Config, PyK4A
from pyk4a import PyK4APlayback, CalibrationType
import open3d as o3d
import numpy as np
import os
import plotter as PLOT
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins'
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pandas as pd
import cv2
import matplotlib.pyplot as plt
import ctypes
from scipy.spatial.transform import Rotation as R
import pyk4a
import matrix_utilities as MATRIX_UTILS
import PREPROC_double_KA as KA_PREPROC
import re
import sys
from decorator import *
import localization_proc as LOCALIZE


#TODO in fusione fai in modo che l area presa in considerazione sia un multiplo
# dell area comune che non sai qual e, ma tieni conto che sia circa 3 metri,
# quinid prendi solo l area - 3 metri fdal centro di fusione,
# che dovrebbe essere 95 perc source, 0 percentile target o simile sulla x, e 50 per y
# TIRA GIU anche l accelerometro della kinect

def create_axes(length=1.0):
    # Asse X - Rosso
    box_x = o3d.geometry.TriangleMesh.create_box(width=length, height=0.02, depth=0.02)
    box_x.paint_uniform_color([1, 0, 0])  # Colore rosso per l'asse X
    box_x.translate([length / 2, 0, 0])  # Centrato sull'origine

    # Asse Y - Verde
    box_y = o3d.geometry.TriangleMesh.create_box(width=0.02, height=length, depth=0.02)
    box_y.paint_uniform_color([0, 1, 0])  # Colore verde per l'asse Y
    box_y.translate([0, length / 2, 0])  # Centrato sull'origine

    # Asse Z - Blu
    box_z = o3d.geometry.TriangleMesh.create_box(width=0.02, height=0.02, depth=length)
    box_z.paint_uniform_color([0, 0, 1])  # Colore blu per l'asse Z
    box_z.translate([0, 0, length / 2])  # Centrato sull'origine

    return [box_x, box_y, box_z]
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

    axes = create_axes(length=1.0)
    for axis in axes:
        vis.add_geometry(axis)


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




def weighted_average_quaternions(q1, q2, w1, w2):
    """
    Esegue una media pesata tra due quaternioni e normalizza il risultato.
    """
    q = w1 * q1 + w2 * q2
    return q / np.linalg.norm(q)


def delta_angle(angle1, angle2):
    delta = angle2 - angle1
    return (delta + np.pi) % (2 * np.pi) - np.pi

def correct_slam_with_gnssimu(previous_timestamp, downsampled_data, current_timestamp, updated_trasform_icp_result, w_icp, w_gnss, no_pc=False):

    if previous_timestamp is not None:
        # Estrarre la trasformazione corrispondente dalla traiettoria GNSS e IMU
        current_trajectory = downsampled_data.get(current_timestamp)
        previous_trajectory = downsampled_data.get(previous_timestamp)

        # Calcolare il delta della traiettoria tra il timestamp corrente e quello precedente
        delta_position = {
            'x': current_trajectory['position']['x'] - previous_trajectory['position']['x'],
            'y': current_trajectory['position']['y'] - previous_trajectory['position']['y'],
            'z': current_trajectory['position']['z'] - previous_trajectory['position']['z']
        }
        delta_rotation = {
            'x': delta_angle(previous_trajectory['rotation']['x'], current_trajectory['rotation']['x']),
            'y': delta_angle(previous_trajectory['rotation']['y'], current_trajectory['rotation']['y']),
            'z': delta_angle(previous_trajectory['rotation']['z'], current_trajectory['rotation']['z'])
        }




        if no_pc:
            #print(" NO SLAM")
            # Utilizza direttamente la trasformazione GNSS/IMU senza mediazione
            # Crea le rotazioni singolarmente
            rot_x = R.from_euler('x', delta_rotation['x'], degrees=False).as_matrix()
            rot_y = R.from_euler('y', delta_rotation['y'], degrees=False).as_matrix()
            rot_z = R.from_euler('z', delta_rotation['z'], degrees=False).as_matrix()


            # rot_x = R.from_euler('x', delta_rotation['x'], degrees=False).as_matrix()
            # rot_y = R.from_euler('y', 0, degrees=False).as_matrix()
            # rot_z = R.from_euler('z', 0, degrees=False).as_matrix()


            # Combina le rotazioni nell'ordine desiderato per una rotazione estrinseca
            # (rotazione globale nell'ordine x, poi y, poi z)
            combined_rotation_matrix = rot_x @ rot_y @ rot_z






            combined_translation = np.array([delta_position['x'], delta_position['y'], delta_position['z']])
        else:
            # Converti la matrice di rotazione in quaternione per l'ICP
            icp_rotation_matrix = np.array(updated_trasform_icp_result[:3, :3], copy=True)
            icp_quaternion = R.from_matrix(icp_rotation_matrix).as_quat()

            # Converti la rotazione GNSS/IMU in quaternione
            gnss_rotation = R.from_euler('xyz', [delta_rotation['x'], delta_rotation['y'], delta_rotation['z']], degrees=False)
            gnss_quaternion = gnss_rotation.as_quat()

            # Calcola la media pesata dei quaternioni
            combined_quaternion = w_icp * icp_quaternion + w_gnss * gnss_quaternion
            combined_quaternion /= np.linalg.norm(combined_quaternion)

            # Converti il quaternione mediato in una matrice di rotazione
            combined_rotation_matrix = R.from_quat(combined_quaternion).as_matrix()

            # Calcola la traslazione mediata
            delta_translation_icp = updated_trasform_icp_result[:3, 3]
            delta_translation_gnss = np.array([delta_position['x'], delta_position['y'], delta_position['z']])
            combined_translation = w_icp * delta_translation_icp + w_gnss * delta_translation_gnss

        # Ricostruisci la matrice di trasformazione 4x4
        combined_transformation = np.eye(4)
        combined_transformation[:3, :3] = combined_rotation_matrix
        combined_transformation[:3, 3] = combined_translation
    else:

        combined_transformation = np.eye(4)
    return combined_transformation


def incremental_plot(downsampled_data):
    """
    Crea un grafico 2x3 con gli incrementi di posizione e rotazione
    dai dati di downsampled_data.

    Parameters:
    downsampled_data (dict): Dizionario con timestamp come chiavi e valori di posizione e rotazione
                             per ciascun asse.
    """


    # Funzione per calcolare il delta angolare continuo per evitare discontinuità
    def delta_angle(angle1, angle2):
        delta = angle2 - angle1
        return (delta + np.pi) % (2 * np.pi) - np.pi

    # Inizializzare liste per gli incrementi di posizione e rotazione
    delta_positions = {'x': [], 'y': [], 'z': []}
    delta_rotations = {'x': [], 'y': [], 'z': []}

    # Ottenere il primo elemento per iniziare il calcolo degli incrementi
    timestamps = sorted(downsampled_data.keys())
    previous_trajectory = downsampled_data[timestamps[0]]

    # Calcolare gli incrementi tra i timestamp consecutivi
    for t in timestamps[1:]:
        current_trajectory = downsampled_data[t]

        # Calcolare l'incremento di posizione
        delta_positions['x'].append(current_trajectory['position']['x'] - previous_trajectory['position']['x'])
        delta_positions['y'].append(current_trajectory['position']['y'] - previous_trajectory['position']['y'])
        delta_positions['z'].append(current_trajectory['position']['z'] - previous_trajectory['position']['z'])

        # Calcolare l'incremento di rotazione utilizzando la funzione delta_angle
        delta_rotations['x'].append(
            delta_angle(previous_trajectory['rotation']['x'], current_trajectory['rotation']['x']))
        delta_rotations['y'].append(
            delta_angle(previous_trajectory['rotation']['y'], current_trajectory['rotation']['y']))
        delta_rotations['z'].append(
            delta_angle(previous_trajectory['rotation']['z'], current_trajectory['rotation']['z']))

        previous_trajectory = current_trajectory

    # Plotting degli incrementi
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Incremental Plot of Position and Rotation")

    # Plot degli incrementi di posizione
    axs[0, 0].plot(delta_positions['x'], label="Delta X Position")
    axs[0, 0].set_title("Delta X Position")
    axs[0, 0].legend()

    axs[0, 1].plot(delta_positions['y'], label="Delta Y Position")
    axs[0, 1].set_title("Delta Y Position")
    axs[0, 1].legend()

    axs[0, 2].plot(delta_positions['z'], label="Delta Z Position")
    axs[0, 2].set_title("Delta Z Position")
    axs[0, 2].legend()

    # Plot degli incrementi di rotazione
    axs[1, 0].plot(delta_rotations['x'], label="Delta X Rotation")
    axs[1, 0].set_title("Delta X Rotation")
    axs[1, 0].legend()

    axs[1, 1].plot(delta_rotations['y'], label="Delta Y Rotation")
    axs[1, 1].set_title("Delta Y Rotation")
    axs[1, 1].legend()

    axs[1, 2].plot(delta_rotations['z'], label="Delta Z Rotation")
    axs[1, 2].set_title("Delta Z Rotation")
    axs[1, 2].legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout to include the main title
    plt.show()


def filter_points_within_radius(pointcloud, center, radius):
    points = np.asarray(pointcloud.points)
    distances = np.linalg.norm(points - center, axis=1)
    filtered_points = points[distances <= radius]
    filtered_pointcloud = o3d.geometry.PointCloud()
    filtered_pointcloud.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_pointcloud, len(points) - len(filtered_points)
def flatten_first_level(nested_list):
    result = []
    for sublist in nested_list:
        flattened_sublist = []
        for item in sublist:
            if isinstance(item, list):
                flattened_sublist.extend(flatten_first_level([item])[0])
            else:
                flattened_sublist.append(item)
        result.append(flattened_sublist)
    return result

@timeit
def hierarchy_slam_icp(input_all_pointcloud, timestamp_sorted,downsampled_data, VOXEL_VOLUME,w_icp=0.9, w_gnss=0.1):
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

    #incremental_plot(downsampled_data)




    # al primo giro prendo PC grezze
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0
    last_epoch_trasformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]
    x_y = []
    timestamp_dict = {}
    bernardo_timestamp_dictionary = {}
    bernardo_timestamp_list = []


    max_filtered_pointcloud_size = 0
    total_filtered_points = 0

    trajectory_deltas = {}

    while len(epoch_start_pointclouds) > 1:
        start_time = time.time()
        epoch += 1
        i = 1

        new_halfed_pointcloud = []

        trasformation_current = []
        # Dizionario per i timestamp delle nuove pointcloud fuse

        print(f"START Epoch {epoch}, PCs:{len(epoch_start_pointclouds)}")
        bernardo_empty_list_timestamp = []

        while i < len(epoch_start_pointclouds):
            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(len(last_epoch_trasformation))
                print(len(epoch_start_pointclouds))

            if i > 0:
                initial_trasform_from_last_epoch_1_couple = (last_epoch_trasformation[i - 1])
                initial_trasform_from_last_epoch_2_couple = (last_epoch_trasformation[i])

                prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)
                source_raw = epoch_start_pointclouds[i]

                current_source = o3d.geometry.PointCloud(source_raw)
                current_source.transform(initial_trasform_from_last_epoch_1_couple)
                target = epoch_start_pointclouds[i - 1]

                if epoch == 1:
                    bernardo_timestamp_list.append([timestamp_sorted[i-1],timestamp_sorted[i]])
                    current_timestamp = timestamp_sorted[i]
                    previous_timestamp = timestamp_sorted[i - 1]
                else:
                    bernardo_empty_list_timestamp.append([bernardo_timestamp_list[i-1],bernardo_timestamp_list[i]])
                    current_timestamp = min(bernardo_timestamp_list[i])
                    previous_timestamp = max(bernardo_timestamp_list[i-1])



                # Timestamp di riferimento: più recente del source e precedente del target



                # Aggiorna la mappa dei timestamp per la nuova pointcloud fusa



                if epoch < 3:

                    if len(current_source.points) > 15000:
                        current_source = downsample_point_cloud(current_source, VOXEL_VOLUME)
                    if len(target.points) > 15000:
                        target = downsample_point_cloud(target, VOXEL_VOLUME)

                    filtered_source = current_source
                    filtered_target = target
                else:

                    current_source = downsample_point_cloud(current_source, VOXEL_VOLUME)
                    target = downsample_point_cloud(target, VOXEL_VOLUME)

                    source_bounds = np.asarray(current_source.get_max_bound()) - np.asarray(
                        current_source.get_min_bound())
                    target_bounds = np.asarray(target.get_max_bound()) - np.asarray(target.get_min_bound())
                    max_source_bound = np.max(source_bounds)
                    max_target_bound = np.max(target_bounds)
                    # print(f"MaxDIM: {round(np.max([max_source_bound, max_target_bound]), 2)} m")

                    if np.any(source_bounds > 8) or np.any(target_bounds > 8):

                        source_centroid = np.mean(np.asarray(current_source.points), axis=0)
                        target_centroid = np.mean(np.asarray(target.points), axis=0)
                        central_point = (source_centroid + target_centroid) / 2

                        radius = 4.0
                        filtered_source, filtered_points_source = filter_points_within_radius(current_source,
                                                                                              central_point, radius)
                        source_bounds_filtered = np.asarray(filtered_source.get_max_bound()) - np.asarray(
                            filtered_source.get_min_bound())
                        max_source_filtered_bound = np.max(source_bounds_filtered)

                        filtered_target, filtered_points_target = filter_points_within_radius(target, central_point,
                                                                                              radius)
                        target_bounds_filtered = np.asarray(filtered_target.get_max_bound()) - np.asarray(
                            filtered_target.get_min_bound())
                        max_target_filtered_bound = np.max(target_bounds_filtered)
                        print(
                            f"TENDAGGIO SOURCE from {round(max_source_bound, 2)} to {round(max_source_filtered_bound, 2)} m (-{filtered_points_source} points)")
                        print(
                            f"TENDAGGIO TARGET from {round(max_target_bound, 2)} to {round(max_target_filtered_bound, 2)} m (-{filtered_points_target} points)")

                        total_filtered_points += filtered_points_source + filtered_points_target
                        max_filtered_pointcloud_size = max(max_filtered_pointcloud_size, len(filtered_source.points),
                                                           len(filtered_target.points))
                    else:
                        filtered_source = current_source
                        filtered_target = target









                if 1:
                #if len(filtered_source.points) < 200 or len(filtered_target.points) < 200:
                    updated_trasform_icp_result = correct_slam_with_gnssimu(previous_timestamp, downsampled_data,
                                                                            current_timestamp,
                                                                            0, w_icp,
                                                                            w_gnss, True)
                else:
                    updated_trasform_icp_result_raw = CUSTOM_SLAM.icp_open3d(filtered_source, filtered_target, 0.05, 200)
                    updated_trasform_icp_result = correct_slam_with_gnssimu(previous_timestamp, downsampled_data,
                                                                            current_timestamp,
                                                                            updated_trasform_icp_result_raw, w_icp,
                                                                            w_gnss)

                if previous_timestamp is not None:
                    # Estrarre la trasformazione corrispondente dalla traiettoria GNSS e IMU
                    current_trajectory = downsampled_data.get(current_timestamp)
                    previous_trajectory = downsampled_data.get(previous_timestamp)

                    # Calcolare il delta della traiettoria tra il timestamp corrente e quello precedente
                    delta_position = {
                        'x': current_trajectory['position']['x'] - previous_trajectory['position']['x'],
                        'y': current_trajectory['position']['y'] - previous_trajectory['position']['y'],
                        'z': current_trajectory['position']['z'] - previous_trajectory['position']['z']
                    }
                    delta_rotation = {
                        'x': current_trajectory['rotation']['x'] - previous_trajectory['rotation']['x'],
                        'y': current_trajectory['rotation']['y'] - previous_trajectory['rotation']['y'],
                        'z': current_trajectory['rotation']['z'] - previous_trajectory['rotation']['z']
                    }
                else:
                    # Se non c'è una traiettoria precedente, il delta è considerato zero
                    delta_position = {'x': 0, 'y': 0, 'z': 0}
                    delta_rotation = {'x': 0, 'y': 0, 'z': 0}

                # Creare un dizionario per la fusione corrente con la trasformazione stimata dallo SLAM ICP e il delta GNSS/IMU
                trajectory_deltas[current_timestamp] = {
                    'slam_icp_transformation': {
                        'delta_x': updated_trasform_icp_result[0, 3],
                        'delta_y': updated_trasform_icp_result[1, 3],
                        'delta_z': updated_trasform_icp_result[2, 3],
                        'rotation_matrix': updated_trasform_icp_result[:3, :3].tolist()
                    },
                    'gnss_imu_transformation': {
                        'delta_x': delta_position['x'],
                        'delta_y': delta_position['y'],
                        'delta_z': delta_position['z'],
                        'delta_rotation_x': delta_rotation['x'],
                        'delta_rotation_y': delta_rotation['y'],
                        'delta_rotation_z': delta_rotation['z']
                    }
                }

                # Estrarre il timestamp associato


                # Estrarre la trasformazione corrispondente dalla traiettoria GNSS e IMU
                current_trajectory = downsampled_data.get(current_timestamp)
                previous_trajectory = downsampled_data[previous_timestamp] if previous_timestamp in downsampled_data else None

                if previous_trajectory is not None:
                    # Calcolare il delta della traiettoria tra il timestamp corrente e quello precedente
                    delta_position = {
                        'x': current_trajectory['position']['x'] - previous_trajectory['position']['x'],
                        'y': current_trajectory['position']['y'] - previous_trajectory['position']['y'],
                        'z': current_trajectory['position']['z'] - previous_trajectory['position']['z']
                    }
                    delta_rotation = {
                        'x': current_trajectory['rotation']['x'] - previous_trajectory['rotation']['x'],
                        'y': current_trajectory['rotation']['y'] - previous_trajectory['rotation']['y'],
                        'z': current_trajectory['rotation']['z'] - previous_trajectory['rotation']['z']
                    }
                else:
                    # Se non c'è una traiettoria precedente, il delta è considerato zero
                    delta_position = {'x': 0, 'y': 0, 'z': 0}
                    delta_rotation = {'x': 0, 'y': 0, 'z': 0}

                # Creare un dizionario per la fusione corrente con la trasformazione stimata dallo SLAM ICP e il delta GNSS/IMU
                trajectory_deltas[current_timestamp] = {
                    'slam_icp_transformation': {
                        'delta_x': updated_trasform_icp_result[0, 3],
                        'delta_y': updated_trasform_icp_result[1, 3],
                        'delta_z': updated_trasform_icp_result[2, 3],
                        'delta_rotation_x': np.arctan2(updated_trasform_icp_result[2, 1], updated_trasform_icp_result[2, 2]),
                        'delta_rotation_y': np.arcsin(-updated_trasform_icp_result[2, 0]),
                        'delta_rotation_z': np.arctan2(updated_trasform_icp_result[1, 0], updated_trasform_icp_result[0, 0]),
                        'rotation_matrix': updated_trasform_icp_result[:3, :3].tolist()
                    },
                    'gnss_imu_transformation': {
                        'delta_x': delta_position['x'],
                        'delta_y': delta_position['y'],
                        'delta_z': delta_position['z'],
                        'delta_rotation_x': delta_rotation['x'],
                        'delta_rotation_y': delta_rotation['y'],
                        'delta_rotation_z': delta_rotation['z']
                    }
                }
                x_y.append(updated_trasform_icp_result[0, -3:])
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)

                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                # Creazione degli assi di riferimento



                merged_pcd = target + trasformed_icp_source
                merged_pcd = remove_isolated_points(merged_pcd, nb_neighbors=12, radius=0.4)


                trasformation_current.append(total_trasformation_prior_plus_icp)
                new_halfed_pointcloud.append(merged_pcd)

                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:
                    print("DISPARI")
                    print(f"START Epoch {epoch},i = {i}, PCs AVIABLE:{len(epoch_start_pointclouds)} PCS comp:{len(new_halfed_pointcloud)} trasform:{len(trasformation_current)}")
                    print("add last PC with its trasformation")
                    last_pc = epoch_start_pointclouds[i + 1]
                    if epoch == 1:
                        bernardo_timestamp_list.append([timestamp_sorted[i+1]])
                    else:
                        bernardo_empty_list_timestamp.append(bernardo_timestamp_list[i+1])


                    last_trasaform = (last_epoch_trasformation[i + 1])




                    trasformation_current.append(last_trasaform)
                    new_halfed_pointcloud.append(last_pc)




            i += 2

        print("LEN pc and ts : ", len(new_halfed_pointcloud), len(bernardo_empty_list_timestamp))

        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_trasformation = trasformation_current

        if epoch > 1:
            bernardo_timestamp_list = flatten_first_level(bernardo_empty_list_timestamp)
        print("LEN SINGLE TS ELEMENT : ", len(bernardo_timestamp_list[0]))
        end_time = time.time()
        elapsed_time = end_time - start_time
        elapsed_time_ms = round(elapsed_time, 3)
        print(f"Computed Epoch {epoch} in {elapsed_time_ms} s, PCs and T created:{len(new_halfed_pointcloud)}")



    x_values = [item[0] for item in x_y]
    y_values = [item[1] for item in x_y]
    z_values = [item[2] for item in x_y]



    return new_halfed_pointcloud[0] , trajectory_deltas



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
        interpolated_gnss_data , gnss_timestamps, trajectory_gnss = LOCALIZE.interpolate_gnss_for_imu(gnss_data, imu_data)
        interpolated_gnss_data = LOCALIZE.align_gnss_trajectory(interpolated_gnss_data)


        #LOCALIZE.plot_gnss_data(interpolated_gnss_data)

        #LOCALIZE.scykit_process_imu(imu_file)

        timestamps_imu, trajectory_imu, linear_accelerations, global_accelerations, rotations = LOCALIZE.process_imu_data(imu_file)

        #LOCALIZE.plot_translations_double_confront(gnss_timestamps, trajectory_gnss, timestamps_imu, trajectory_imu)


        #CON ROTAZIONI IMU
        #interpolated_data = LOCALIZE.interpolate_imu_gnss(timestamps_imu,rotations,interpolated_gnss_data)
        #ROTAZIONI GENENREATE  DA TRAIETTORIA
        interpolated_data = LOCALIZE.generate_gnss_based_orientation(interpolated_gnss_data)


        # Plot the translations


        # Plot the linear accelerationsa
        # LOCALIZE.plot_accelerations(timestamps, linear_accelerations)
        # LOCALIZE.plot_accelerations(timestamps, global_accelerations)
        # LOCALIZE.plot_accelerations_and_rotations(timestamps, linear_accelerations, rotations)
        # LOCALIZE.plot_accelerations_and_rotations(timestamps, global_accelerations,rotations)

        # # Plot the rotations
        # LOCALIZE.plot_rotations(timestamps, rotations)

        # Plot the 3D trajectory
        #LOCALIZE.plot_trajectory_3d_imuu(timestamps_imu, trajectory_imu, rotations)







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
    starts = 1900
    ends = 2500
    # starts = 500
    # ends = 750
    voxel = 0.05

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

    #SE FREQUENZA GNSSIMU>SLAM
    #downsampled_gnss_imu = LOCALIZE.downsample_interpolated_data_to_slam(timestamp_sorted,interpolated_data)
    #IMU NON APPLICATO - FRQUENZA NATURALE GNSS < SLAM
    downsampled_gnss_imu = LOCALIZE.resample_data_to_slam_frequency(timestamp_sorted,interpolated_data)



    if 0:
        PLOT.plot_GNSS_slamFPS_trajectories(downsampled_gnss_imu)
        PLOT.plot_interpolated_data(interpolated_data)
        PLOT.plot_3d_trajectory_with_arrows(interpolated_data)



    fused, trajectory_deltas = hierarchy_slam_icp(pointclouds,timestamp_sorted, downsampled_gnss_imu, voxel)
    filename = "output_double/fused.ply"
    o3d.io.write_point_cloud(filename, fused)
    print(f"PointCloud salvata come {filename}")





    # now coupled
    print("COUPLED TIMESTAMP")
    PLOT.plot_trajectory_2d_FROM_COUPLED_DELTAS(trajectory_deltas)
    PLOT.plot_3d_trajectory_from_coupled_deltas(trajectory_deltas)
    PLOT.plot_angles_2d_FROM_COUPLED_DELTAS(trajectory_deltas)
    PLOT.visualize_pc_with_trajectory(fused, trajectory_deltas)

    # visualize_pc(fused,"end")

    sys.exit()
