import sys
from matrix_utilities import *
import numpy as np
import os
import open3d as o3d
from trasform_KINECT import *

import os
import re
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import pickle


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

                prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)

                source_raw = epoch_start_pointclouds[i]
                # devo aggiungere anche la trasformata delle iterazioni prima BASTA CHE LE LAST EPOCH LA COMPRENDANO


                current_source = o3d.geometry.PointCloud(source_raw)
                current_source.transform(prior_trasformation_composed)

                target =  epoch_start_pointclouds[i-1]
                updated_trasform_icp_result = icp_open3d(current_source, target)

                #Total trasformation : current, plus the prrevious frame trasf:
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)


                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                merged_pcd = target + trasformed_icp_source
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
        print(f"Computed Epoch {epoch}, PCs created:{len(new_halfed_pointcloud)}")




        dict_analysis[epoch] = [trasformation_current , new_halfed_pointcloud]


    return new_halfed_pointcloud[0]

def plot_interpolated_matrices(rot_dict, trans_dict, show_points=True, show_arrows=True, arrow_scale=0.5):
    fig, ax = plt.subplots(figsize=(10,12))
    i = 0

    yaw = []
    pitch  = [] # no pitch
    roll  =[] #no roll

    combined_matrices = {}

    for timestamp in rot_dict:
        i += 1

        if timestamp in trans_dict:


            trans_matrix = np.array(trans_dict[timestamp])
            rot_matrix = np.array(rot_dict[timestamp])

            translation = trans_matrix[:3, 3]
            # Extract the x and y components
            x, y = translation[0], translation[1]


            # Estrarre la matrice di rotazione 3x3
            rotation_matrix_3x3 = rot_matrix[:3, :3]
            # Se la IMU era montata sottosopra lungo l'asse Y, dobbiamo invertire il segno della seconda riga



            identity = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])

            rotation_180_yaround = np.array([
                [-1, 0, 0],
                [0, 1, 0],
                [0, 0, -1]
            ])
            rotation_180_xaround = np.array([
                [-1, 0, 0],
                [0, 1, 0],
                [0, 0, -1]
            ])
            rotation_180_zaround = np.array([
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, 1]
            ])

            rotation_180_all = np.array([
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1]
            ])

            # Esempio di utilizzo
            rot_matrix_CS = np.array([
                [0, -1, 0],
                [1, 0, 0],
                [0, 0, 1]
            ])

            # Applica la matrice di inversione alla matrice di rotazione originale
            #corrected_rotation_matrix = np.dot(rotation_matrix_3x3, rotation_180_xaround)

            #corrected_rotation_matrix = cambia_segno_angolo_z(rotation_matrix_3x3)

            corrected_rotation_matrix = rotation_matrix_3x3




            #corrected_rotation_matrix = rotation_matrix_3x3

            #corrected_rotation_matrix = np.dot(total_y, rotation_matrix_3x3)


            # Estrarre la sotto-matrice 2x2 che rappresenta la rotazione intorno all'asse Z
            rotation_matrix_z = corrected_rotation_matrix[:2, :2]

            # Estrarre il vettore di orientamento (prima colonna della matrice di rotazione Z)
            orientation = rotation_matrix_z[:, 0]


            # Definire l'origine e la fine della freccia basata sull'orientamento

            end = orientation

            # Estrarre il vettore di orientamento per gli assi X e Y
            orientation_x = corrected_rotation_matrix[:, 0]  # Asse X
            orientation_y = corrected_rotation_matrix[:, 1]  # Asse Y

            # Definire l'origine e le frecce basate sull'orientamento
            origin = np.array([0, 0])
            end_x = orientation_x[:2]
            end_y = orientation_y[:2]

            combined_matrix = np.eye(4)
            combined_matrix[:3, :3] = corrected_rotation_matrix
            combined_matrix[:3, 3] = translation

            combined_matrices[timestamp] = combined_matrix

            # PLOT ANGLE

            angles = extract_euler_angles(corrected_rotation_matrix)
            euler_angles_degrees = np.degrees(angles)

            # Accumulare gli angoli in liste usando un ciclo for

            yaw.append(euler_angles_degrees[2])
            pitch.append(euler_angles_degrees[1])
            roll.append(euler_angles_degrees[0])


            if i % 70 == 0:

                if show_points:
                    if i < 80:
                        ax.text(x + 2, y  + 2, "START", fontsize=8, ha='center')

                    ax.scatter(x, y,color = 'black')

                #roatzione di PI, cambio segno al seno
                if show_arrows:
                    ax.arrow(x,y, end_x[0], end_x[1], head_width=0.8, head_length=2, fc='red',
                             ec='red', label='Asse X')
                    ax.arrow(x,y, end_y[0], end_y[1], head_width=0.8, head_length=2, fc='blue',
                             ec='blue', label='Asse Y')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('IMU Translation and Orientation')
    ax.axis('equal')

    # Creazione di oggetti proxy per la legenda
    red_arrow = plt.Line2D([0], [0], color='red', marker='>', linestyle='None', markersize=10, label='Asse X')
    blue_arrow = plt.Line2D([0], [0], color='blue', marker='>', linestyle='None', markersize=10, label='Asse Y')

    # Aggiunta della legenda
    ax.legend(handles=[red_arrow, blue_arrow])

    plt.show()
    PLOT_ANGLE = 0
    if PLOT_ANGLE:
        # Plotare solo l'angolo di yaw
        plt.figure()
        plt.plot(yaw, marker='o', linestyle='-', color='blue')
        plt.title('Yaw Angle over Time')
        plt.xlabel('Time')
        plt.ylabel('Yaw Angle (degrees)')
        plt.grid(True)
        plt.show()



    return combined_matrices





def interpolate_imu_with_PC(pointcloud_dir, imu_dict):
    pointcloud_files = [f for f in os.listdir(pointcloud_dir) if f.endswith('.ply')]
    pointcloud_timestamps = []

    # Extract timestamps from filenames
    for file in pointcloud_files:
        match = re.search(r'(\d+\.\d+)', file)
        if match:
            timestamp = float(match.group(1))
            pointcloud_timestamps.append(timestamp)

    # Sort the pointcloud timestamps
    pointcloud_timestamps.sort()

    imu_timestamps = list(imu_dict.keys())
    imu_timestamps.sort()

    # Convert string timestamps to floats
    imu_timestamps_float = [float(ts) for ts in imu_timestamps]

    interpolated_matrices = {}

    for pc_timestamp in pointcloud_timestamps:
        # Find the indices for interpolation
        for i in range(len(imu_timestamps_float) - 1):
            if imu_timestamps_float[i] <= pc_timestamp <= imu_timestamps_float[i + 1]:
                t0 = imu_timestamps_float[i]
                t1 = imu_timestamps_float[i + 1]
                T0 = np.array(imu_dict[imu_timestamps[i]])
                T1 = np.array(imu_dict[imu_timestamps[i + 1]])

                # Interpolate rotation matrices using slerp (spherical linear interpolation)
                r0 = R.from_matrix(T0[:3, :3])
                r1 = R.from_matrix(T1[:3, :3])
                slerp = Slerp([t0, t1], R.from_matrix([T0[:3, :3], T1[:3, :3]]))
                r_interp = slerp([pc_timestamp]).as_matrix()[0]

                # Interpolate translation vectors linearly (though they are zero in this case)
                translation = (T1[:3, 3] - T0[:3, 3]) * ((pc_timestamp - t0) / (t1 - t0)) + T0[:3, 3]

                # Create the interpolated transformation matrix
                T_interp = np.eye(4)
                T_interp[:3, :3] = r_interp
                T_interp[:3, 3] = translation

                interpolated_matrices[str(pc_timestamp)] = T_interp
                break

    return interpolated_matrices



def plot_trajectories(transformations1, transformations2):
    """
    Plotta le traiettorie assolute per le componenti x, y, z e le rotazioni roll, pitch, yaw delle due liste di trasformazioni.

    Args:
        transformations1 (list of np.ndarray): Prima lista di matrici di trasformazione.
        transformations2 (list of np.ndarray): Seconda lista di matrici di trasformazione.
    """

    # Calcolare le coordinate assolute
    x1, y1, z1 = extract_positions(transformations1)
    x2, y2, z2 = extract_positions(transformations2)

    # Calcolare le rotazioni
    roll1, pitch1, yaw1 = extract_rotations(transformations1)
    roll2, pitch2, yaw2 = extract_rotations(transformations2)

    # Creare i plot delle traiettorie e delle rotazioni
    fig = plt.figure(figsize=(18, 12))

    # Plot per x
    ax1 = fig.add_subplot(231)
    ax1.plot(x1, label='gnss 1')
    ax1.plot(x2, label='slam 2')
    ax1.set_xlabel('Indice')
    ax1.set_ylabel('X')
    ax1.set_title('Traiettoria X')
    ax1.legend()
    ax1.grid()

    # Plot per y
    ax2 = fig.add_subplot(232)
    ax2.plot(y1, label='gnss 1')
    ax2.plot(y2, label='slam 2')
    ax2.set_xlabel('Indice')
    ax2.set_ylabel('Y')
    ax2.set_title('Traiettoria Y')
    ax2.legend()
    ax2.grid()

    # Plot per z
    ax3 = fig.add_subplot(233)
    ax3.plot(z1, label='gnss 1')
    ax3.plot(z2, label='slam 2')
    ax3.set_xlabel('Indice')
    ax3.set_ylabel('Z')
    ax3.set_title('Traiettoria Z')
    ax3.legend()
    ax3.grid()

    # Plot per roll
    ax4 = fig.add_subplot(234)
    ax4.plot(roll1, label='gnss 1')
    ax4.plot(roll2, label='slam 2')
    ax4.set_xlabel('Indice')
    ax4.set_ylabel('Roll')
    ax4.set_title('Rotazione Roll')
    ax4.legend()
    ax4.grid()

    # Plot per pitch
    ax5 = fig.add_subplot(235)
    ax5.plot(pitch1, label='gnss 1')
    ax5.plot(pitch2, label='slam 2')
    ax5.set_xlabel('Indice')
    ax5.set_ylabel('Pitch')
    ax5.set_title('Rotazione Pitch')
    ax5.legend()
    ax5.grid()

    # Plot per yaw
    ax6 = fig.add_subplot(236)
    ax6.plot(yaw1, label='gnss 1')
    ax6.plot(yaw2, label='slam 2')
    ax6.set_xlabel('Indice')
    ax6.set_ylabel('Yaw')
    ax6.set_title('Rotazione Yaw')
    ax6.legend()
    ax6.grid()

    plt.tight_layout()
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



# Funzione per creare una point cloud rappresentante un piano traslato e ruotato
def create_transformed_plane(x_shift=0.0, y_shift=0.0, z_shift=0.0, rotation_angle=0.0):
    # Creare una griglia di punti su un piano
    xx, yy = np.meshgrid(np.arange(-1, 1, 0.1), np.arange(-1, 1, 0.1))
    zz = np.zeros_like(xx)
    points = np.vstack((xx.flatten(), yy.flatten(), zz.flatten())).T

    # Applicare la traslazione
    points[:, 0] += x_shift
    points[:, 1] += y_shift
    points[:, 2] += z_shift

    # Applicare la rotazione intorno all'asse Z
    rotation_matrix = np.array([
        [np.cos(rotation_angle), -np.sin(rotation_angle), 0],
        [np.sin(rotation_angle), np.cos(rotation_angle), 0],
        [0, 0, 1]
    ])
    points = points @ rotation_matrix.T

    # Creare una point cloud da questi punti
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


# # Creare una serie di point cloud rappresentanti piani traslati e ruotati
# n_planes = 10
# shift_per_plane = 0.10
# rotation_per_plane = np.pi / 18  # 10 gradi
# pointclouds = []
#
# for i in range(n_planes):
#     pcd = create_transformed_plane(
#         x_shift=i * (shift_per_plane + 0.5),
#         y_shift=i * shift_per_plane,
#         z_shift=i * shift_per_plane,
#         rotation_angle=i * rotation_per_plane
#     )
#     pointclouds.append(pcd)


# Funzione per eseguire ICP con Open3D
# def icp_open3d(source, target, threshold = 0.01 , max_iterations=250):
#     initial_transformation = np.eye(4)
#       # Soglia di distanza per ICP
#     icp_result = o3d.pipelines.registration.registration_icp(
#         source, target, threshold, initial_transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
#     )
#     return icp_result.transformation





def normalize_matrix_trasformation(matrixes):
    # Ottenere la traslazione della prima matrice
    initial_translation = matrixes[0][:3, 3]
    # Creare una lista per le nuove trasformazioni
    normalized_transformations = []
    for transformation in matrixes:
        # Creare una copia della matrice di trasformazione corrente
        new_transformation = np.copy(transformation)
        # Sottrarre la traslazione iniziale
        new_transformation[:3, 3] -= initial_translation
        # Aggiungere la nuova matrice alla lista
        normalized_transformations.append(new_transformation)
    return normalized_transformations


# Funzione per estrarre il timestamp dal nome del file
def get_timestamp(file_name):
    timestamp_str = file_name.split('_')[1].split('.ply')[0]

    return float(timestamp_str)


gnss_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/gnss_data.json'
pointcloud_dir = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak/pc'
imu_orient_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/imu_partial_xsens_data.json'

imu_dict = load_imu_data_from_json(imu_orient_file)
interpolated_imu_raw = interpolate_imu_with_PC(pointcloud_dir,imu_dict)






gnss_array, local_coordinates, reference_latitude, reference_longitude, scale_factor = plot_gnss_trajectory(gnss_file)



interp_lat, interp_lon, interp_alt = interpolate_gnss_data(gnss_array, local_coordinates)
pointcloud_files, transformation_matrices_original,dict_translation = associate_pointclouds_with_gnss(pointcloud_dir, interp_lat, interp_lon,
                                                                            interp_alt, reference_latitude,
                                                                            reference_longitude, scale_factor)


interpolated_imu = normalize_transformation_dictionary(interpolated_imu_raw)
normalize_dictionary_orientation = plot_interpolated_matrices(interpolated_imu, dict_translation)



# Ordinare i file in base al timestamp
pointcloud_files_sorted = sorted(pointcloud_files, key=get_timestamp)



pointclouds = []
only_traslation_matrixes = []
only_rotation_matrixes = []
roto_translation_ground_truth = []
#interst: 1042-1125

print(len(normalize_dictionary_orientation))
print(len(pointcloud_files_sorted))
print(len(dict_translation))

#interst: 1042-1125
start = 2297
timestamp_sorted = []
for idx, file_name in enumerate(pointcloud_files_sorted):
    #if start < idx < 2800:
    #SI PARTE DA i  = 1043
    #print(idx)
    timestamp_str = file_name.split('_')[1].split('.ply')[0]
    timestamp = float(timestamp_str)
    timestamp_sorted.append(timestamp)




    pcd_raw = o3d.io.read_point_cloud(os.path.join(pointcloud_dir, file_name))  # Usa file_name invece di pointcloud_files[idx]
    pcd = remove_isolated_points(pcd_raw)
    pcd_m = convert_to_meters(pcd)
    pointclouds.append(pcd_m)


    translation_matrix = dict_translation[str(timestamp)]
    rotation_only_trans_matrix = normalize_dictionary_orientation[str(timestamp)]

    combined_matrix_total_GT = np.eye(4)
    COMBINE_IMU = 1

    combined_matrix_total_GT[:3, :3] = rotation_only_trans_matrix[:3, :3]  # Imposta la parte di rotazione
    combined_matrix_total_GT[:3, 3] = translation_matrix[:3, 3]


    only_traslation_matrixes.append(translation_matrix)
    roto_translation_ground_truth.append(combined_matrix_total_GT)
    only_rotation_matrixes.append(rotation_only_trans_matrix)



#GNSS dall inizio del tracciato



# per orientazione IMU
#normalize_transformation_list_rot_and_trasl()







lato_a_start = 1129
lato_a_end = 1300
lato_b_start = 2920
lato_b_end = 3120
PC_lato_a = pointclouds[lato_a_start:lato_a_end]
PC_lato_b = pointclouds[lato_b_start:lato_b_end]
timestamp_lato_a = timestamp_sorted[lato_a_start:lato_a_end]
timestamp_lato_b = timestamp_sorted[lato_b_start:lato_b_end]
normalize_roto_trasla_matrix_lato_a = roto_translation_ground_truth[lato_a_start:lato_a_end]
normalize_roto_trasla_matrix_lato_b = roto_translation_ground_truth[lato_b_start:lato_b_end]




# Salva le liste in un file usando pickle
with open('timestamp_and_transformations.pkl', 'wb') as f:
    pickle.dump({
        'timestamp_lato_a': timestamp_lato_a,
        'timestamp_lato_b': timestamp_lato_b,
        'normalize_roto_trasla_matrix_lato_a': normalize_roto_trasla_matrix_lato_a,
        'normalize_roto_trasla_matrix_lato_b': normalize_roto_trasla_matrix_lato_b
    }, f)

print("Liste salvate in timestamp_and_transformations.pkl")



#print(len(normalize_roto_trasla_matrix_lato_a))

#plot_pointcloud_translations(normalize_roto_trasla_matrix_lato_a,normalize_roto_trasla_matrix_lato_b,roto_translation_ground_truth)


#estraggo lato 1 reference
fused_lato_a = hierarchy_slam_icp(PC_lato_a)
#estraggo lato 2 reference
fused_lato_b = hierarchy_slam_icp(PC_lato_b)


save_pointcloud(fused_lato_a, "fused_lato_a_transformed.ply")
save_pointcloud(fused_lato_b, "fused_lato_b_transformed.ply")


point_size = 1
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Merged Point Cloud A')

# Aggiungi le geometrie una alla volta
vis.add_geometry(fused_lato_a)
vis.add_geometry(fused_lato_b)

opt = vis.get_render_option()
opt.point_size = point_size
vis.run()
vis.destroy_window()

sys.exit()



avg_transformation_lato_a = compute_average_transformation(normalize_roto_trasla_matrix_lato_a,(1, 2, 0))
avg_transformation_lato_b = compute_average_transformation(normalize_roto_trasla_matrix_lato_b,(1, 2, 0))

# Applichiamo le trasformazioni medie
fused_lato_a_transformed = apply_transformation(fused_lato_a, avg_transformation_lato_a)
fused_lato_b_transformed = apply_transformation(fused_lato_b, avg_transformation_lato_b)


# Estrai e stampa i valori per lato A
print("Lato A:")
x_a, y_a, z_a, roll_a, pitch_a, yaw_a = extract_position_and_angles(avg_transformation_lato_a)

# Estrai e stampa i valori per lato B
print("Lato B:")
x_b, y_b, z_b, roll_b, pitch_b, yaw_b = extract_position_and_angles(avg_transformation_lato_b)


point_size = 1
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Merged Point Cloud A')

# Aggiungi le geometrie una alla volta
vis.add_geometry(fused_lato_a_transformed)
vis.add_geometry(fused_lato_b_transformed)

opt = vis.get_render_option()
opt.point_size = point_size
vis.run()
vis.destroy_window()

sys.exit()

#sono estratte rispetto all eye!
#tiri fuori anche tutte le timestamp delle pointcloud analizzate (non serve farle entrare nella funzione)


#calcolo posizione media e orientamento medio di entrambe.
#le traslo e le ruoto secondo matrice gnss, per posizonarle ho bisogno però di conoscere le timestamp di riferimento delle stesse
# da timestamp A a timestamp B,-> PC fusa -> estrazone pmatrici posa del ground truth gnss- trasformazione Pc originale tramite matrice media GNSS
#rifusione tra le due PC globalib avvicinate dal GNSS




fused = hierarchy_slam_icp(pointclouds)


point_size = 1
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Merged Point Cloud')
vis.add_geometry(fused)
opt = vis.get_render_option()
opt.point_size = point_size
vis.run()
vis.destroy_window()

sys.exit()



normalized_transformations = normalize_matrix_trasformation(only_traslation_matrixes)
normalize_roto_trasla_matrix = normalize_transformation_list_rot_and_trasl(roto_translation_ground_truth)
normalize_roto_matrix = normalize_transformation_list_rot_and_trasl(only_rotation_matrixes)

absolute_transformations = [normalize_roto_trasla_matrix[0]]  # Lista delle trasformazioni assolute
print("STARTING:",normalize_roto_trasla_matrix[0]) #
processed_pointclouds = [pointclouds[0]] #1043  # La prima point cloud rimane invariata


# absolute_transformations = [np.eye(4)]  # Lista delle trasformazioni assolute
# processed_pointclouds = [pointclouds[0]]

#PARTO DA UNO PERCHE SALTO LA PRIMA
for idx in range(1, len(pointclouds)):
    print(idx,"/",len(pointclouds))
    source = pointclouds[idx] #1044
    target = processed_pointclouds[-1]  # L'ultima point cloud processata

    # Trasformare la point cloud corrente verso la posizione assoluta dell'ultima point cloud trasformata
    #### IMPLEMENTAZIONE MODIFICATA
    # INVECE DI PRENDERE L ULTIMA, FACCIAMO LA MEDIA TRA L ULTIMA E DOVE IL GPS DICE CHE DOVRESTI ESSERE ORA XYZ
    hybrid_absolute_trasform = matrix_mean_with_rotation(absolute_transformations[-1],normalize_roto_trasla_matrix[idx])
    initial_transformation = hybrid_absolute_trasform
    #print(initial_transformation)
    source_transformed = o3d.geometry.PointCloud(source)
    source_transformed.transform(initial_transformation)

    # Eseguire ICP per affinare sia la rotazione che la traslazione
    icp_result = icp_open3d(source_transformed, target)

    # Calcolare la trasformazione assoluta per la point cloud corrente
    absolute_transformation = np.dot(initial_transformation, icp_result)

    absolute_transformations.append(absolute_transformation)

    # Applicare la trasformazione assoluta alla point cloud corrente
    source_temp = o3d.geometry.PointCloud(source)
    source_temp.transform(absolute_transformation)
    processed_pointclouds.append(source_temp)


    # Visualizzare l'allineamento
    # target_temp = target
    # source_temp.paint_uniform_color([0, 1, 0])  # verde
    # target_temp.paint_uniform_color([0.53, 0.81, 0.98])  # blu
    # source_transformed.paint_uniform_color([1, 0, 0]) #rosso
    #o3d.visualization.draw_geometries([source_temp, target_temp, source_transformed], window_name=f'ICP Alignment {idx}')



# Unire tutte le point cloud processate
merged_pointcloud = o3d.geometry.PointCloud()
for pcd in processed_pointclouds:
    merged_pointcloud += pcd

point_size = 1
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='Merged Point Cloud')
vis.add_geometry(merged_pointcloud)
opt = vis.get_render_option()
opt.point_size = point_size
vis.run()
vis.destroy_window()


print(normalized_transformations[-1])
print(absolute_transformations[-1])

#absolute_positions = compute_absolute_positions(absolute_transformations)


#plot_trajectories(normalize_roto_trasla_matrix, absolute_transformations)
plot_pointcloud_translations(normalize_roto_trasla_matrix, absolute_transformations)