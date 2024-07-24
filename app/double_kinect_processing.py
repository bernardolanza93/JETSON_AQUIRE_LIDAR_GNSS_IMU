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

def visualize_pc(pc):
    point_size = 1
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Merged Point Cloud A')

    # Aggiungi le geometrie una alla volta
    vis.add_geometry(pc)


    opt = vis.get_render_option()
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

def remove_isolated_points(pcd, nb_neighbors=10, radius=500):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned


def hierarchy_slam_icp(input_all_pointcloud): #100

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

    while len(epoch_start_pointclouds) > 1:
        epoch += 1


        i = 1


        dict_analysis ={}

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
                updated_trasform_icp_result = CUSTOM_SLAM.icp_open3d(current_source, target)
                #Total trasformation : current, plus the prrevious frame trasf:
                x_y.append(updated_trasform_icp_result[0, -2:])
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)


                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                if len(epoch_start_pointclouds) == 2:
                    print(updated_trasform_icp_result)

                    #pre_icp  = target + current_source
                    #visualize_pc(pre_icp)
                    #visualize_pc(target)

                merged_pcd = target + trasformed_icp_source

                # Controllo se esiste una point cloud precedente e successiva
                # Controllo se esiste una point cloud precedente e successiva

                PREVIOUS_ADDICTION = 0
                if PREVIOUS_ADDICTION:

                    if i - 2 >= 0:
                        previous_pc = epoch_start_pointclouds[i - 2]
                        initial_transform_previous_1 = last_epoch_trasformation[i - 2]
                        initial_transform_previous_2 = last_epoch_trasformation[i - 1]
                        prior_transform_composed_previous = np.dot(initial_transform_previous_1,
                                                                   initial_transform_previous_2)
                        # Trasformare la merged_pcd
                        merged_pcd.transform(prior_transform_composed_previous)
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
                        combined_transform = np.dot(transform_composed_previous, transform_composed_current)
                        combined_transform = np.dot(combined_transform, transform_composed_next)

                        # Trasformare la merged_pcd con le trasformazioni combinate
                        next_pc.transform(combined_transform)

                        # Eseguire ICP per allineare merged_pcd con next_pc
                        next_pc_transformed = o3d.geometry.PointCloud(next_pc)
                        transform_icp_result_next = CUSTOM_SLAM.icp_open3d(merged_pcd, next_pc_transformed)

                        # Correggere merged_pcd con la trasformazione ICP trovata
                        next_pc_transformed.transform(transform_icp_result_next)
                        merged_pcd += next_pc_transformed

                #A QUESTO PUNTO CONTROLLO SE HO UNA PC PRIMA E DOPO,
                #se una delle due manca ( estremi ) ne prenderò solo una, alla fine potrebbe capitare che avendone troppo poche manchino netrmbi (ie quando ho solo 2PC)
                # in questo caso lascierò perdere questa ultiriore fusione dei nuovi estremi

                # l ARREY CHE CONTORLLO È IL SEGUNETE:  epoch_start_pointclouds
                #TARGET E SOURCE SONO RISPETTIVAMENTE i-1 E i
                # i nuovi estremi sono i-2 (previous_pc) e i + 1 (next_pc)
                # dopo
                # SE LE HO DEFINISCO COME POSIZIONARLE TRAMITRE MATRICI :
                # per la pointcloud precedente deovrò posizionarla come avrei fatto nell iterazione prima (e aggiungerla poi a merged Pc dopo che ho fatto icp)
                # per la seguente come avrei fatto per l iterazione dopo.
                #FACCIO ICP E LE FONDO TUTTE E 4,
                # quaale trasformazione prendo : per quella prima: usiamo un sistema sinistroso quindi quella prima parte sempre da zero, in questo caso sposto la merged pc appena ottenuta
                # la sposto
                #in pratica sposto la merged come ho gia spostato la source nell iterazione precedente
                #ovvero usando : (la merged ora si comporta come la source della precedente) quindi vado ad usaree :              initial_trasform_from_last_epoch_1_couple_previous = (last_epoch_trasformation[i-2])
                #initial_trasform_from_last_epoch_2_couple_previous = (last_epoch_trasformation[i-1])
                # prior_trasformation_composed_previous = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)

                # poi facciamo icp, e trasformiamo anche con icp,
                #a  questo punto prendiamo la segunete, merged + previous sono all origine, traslo la next con la matrice specifica, faccio icp,, e traslo con risultati icp
                # traslo semplicemente la source (considerando la merged come target) come avrei fatto nell epoca successiva (e che farò)
                # initial_trasform_from_last_epoch_1_couple_next = (last_epoch_trasformation[i])
                # initial_trasform_from_last_epoch_2_couple_next = (last_epoch_trasformation[i+1])
                #prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)
                # ora faccio icp e poi fondo devo ricordarmi di fondere anche con il previous che aveo attaccato prima non solo con la merge
                # infine fondo anche la next.
                # ora invece di un doppia ho una quadrupla

                # poi salvo solo le info delle due pc centrali non contando le aggiunte che ho fatto nell ambito trasformate (perche una in realta l ho gia computata prima e una la computerò adesso)
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

            i += 2


        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_trasformation = trasformation_current
        print(f"Computed Epoch {epoch}, PCs created:{len(new_halfed_pointcloud)}, trasformation = {len(trasformation_current)} ")

        dict_analysis[epoch] = [trasformation_current , new_halfed_pointcloud]

    # Estraiamo i valori di x e y
    x_values = [item[0] for item in x_y]
    y_values = [item[1] for item in x_y]

    # Creiamo un array di istanti di tempo (può essere una semplice sequenza numerica)
    time_instants = np.arange(len(x_y))

    # Creiamo il grafico
    plt.figure(figsize=(10, 5))

    # Grafico di x vs istanti
    plt.subplot(1, 2, 1)
    plt.plot(time_instants, x_values, marker='o', linestyle='-', color='b', label='x vs istanti')
    plt.xlabel('Istanti di tempo')
    plt.ylabel('x')
    plt.title('x vs Istanti')
    plt.legend()

    # Grafico di y vs istanti
    plt.subplot(1, 2, 2)
    plt.plot(time_instants, y_values, marker='o', linestyle='-', color='r', label='y vs istanti')
    plt.xlabel('Istanti di tempo')
    plt.ylabel('y')
    plt.title('y vs Istanti')
    plt.legend()

    # Mostra il grafico
    plt.tight_layout()
    plt.show()


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
    starts = 2900
    ends = 3200
    timestamp_sorted = []
    pointclouds = []
    for idx, file_name in enumerate(pointcloud_files_sorted):


        timestamp_str = file_name.split('_')[1].split('.ply')[0]
        timestamp = float(timestamp_str)
        timestamp_sorted.append(timestamp)
        if idx > starts and idx < ends:
            pcd_raw = o3d.io.read_point_cloud(
                os.path.join(coupled_saving_folder, file_name))  # Usa file_name invece di pointcloud_files[idx]
            pcd_raw = convert_to_meters(pcd_raw)

            pointclouds.append(pcd_raw)



    fused = hierarchy_slam_icp(pointclouds)
    filename = "output_double/fused.ply"
    o3d.io.write_point_cloud(filename, fused)
    print(f"PointCloud salvata come {filename}")


    visualize_pc(fused)

    sys.exit()
