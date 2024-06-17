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



def plot_interpolated_matrices(rot_dict, trans_dict, show_points=True, show_arrows=True, arrow_scale=0.5):


    fig, ax = plt.subplots()
    i = 0

    yaw = []
    pitch  = [] # no pitch
    roll  =[] #no roll

    for timestamp in rot_dict:
        i += 1

        if timestamp in trans_dict:

            if i % 50 == 0:
                trans_matrix = np.array(trans_dict[timestamp])
                rot_matrix = np.array(rot_dict[timestamp])

                translation = trans_matrix[:3, 3]
                # Extract the x and y components
                x, y = translation[0], translation[1]


                # Estrarre la matrice di rotazione 3x3
                rotation_matrix_3x3 = rot_matrix[:3, :3]
                # Se la IMU era montata sottosopra lungo l'asse Y, dobbiamo invertire il segno della seconda riga


                #rotazione 180 intorno a x
                total = np.array([
                    [1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]
                ])
                total_y = np.array([
                    [-1, 0, 0],
                    [0, 1, 0],
                    [0, 0, -1]
                ])





                corrected_rotation_matrix = np.dot(total_y, rotation_matrix_3x3)
                angles = extract_euler_angles(corrected_rotation_matrix)
                euler_angles_degrees = np.degrees(angles)

                # Accumulare gli angoli in liste usando un ciclo for

                yaw.append(euler_angles_degrees[2])
                pitch.append(euler_angles_degrees[1])
                roll.append(euler_angles_degrees[0])

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





                if show_points:
                    ax.scatter(x, y)

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
    plt.show()

    # Plotare solo l'angolo di yaw
    plt.figure()
    plt.plot(yaw, marker='o', linestyle='-', color='blue')
    plt.title('Yaw Angle over Time')
    plt.xlabel('Time')
    plt.ylabel('Yaw Angle (degrees)')
    plt.grid(True)
    plt.show()





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


def load_imu_data_from_json(json_file):
    with open(json_file, 'r') as f:
        imu_data = json.load(f)

    imu_dict = {}

    for entry in imu_data:
        timestamp = f"{entry['timestamp']['secs']}.{entry['timestamp']['nsecs']:09d}"
        quaternion = (
            entry['orientation']['w'],
            entry['orientation']['x'],
            entry['orientation']['y'],
            entry['orientation']['z']
        )
        rotation_matrix = quaternion_rotation_matrix(quaternion)

        # Create the 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix

        imu_dict[timestamp] = transformation_matrix.tolist()

    return imu_dict





def plot_trajectories(transformations1, transformations2):
    """
    Plotta le traiettorie assolute per le componenti x, y, z delle due liste di trasformazioni.

    Args:
        transformations1 (list of np.ndarray): Prima lista di matrici di trasformazione.
        transformations2 (list of np.ndarray): Seconda lista di matrici di trasformazione.
    """

    # Calcolare le coordinate assolute
    x1,y1,z1 = extract_positions(transformations1)
    x2,y2,z2 = extract_positions(transformations2)

    # Creare i plot delle traiettorie
    fig = plt.figure(figsize=(18, 6))

    # Plot per x
    ax1 = fig.add_subplot(131)
    ax1.plot(x1, label='gnss 1')
    ax1.plot(x2, label='slam 2')
    ax1.set_xlabel('Indice')
    ax1.set_ylabel('X')
    ax1.set_title('Traiettoria X')
    ax1.legend()
    ax1.grid()

    # Plot per y
    ax2 = fig.add_subplot(132)
    ax2.plot(y1, label='gnss 1')
    ax2.plot(y2, label='slam 2')
    ax2.set_xlabel('Indice')
    ax2.set_ylabel('Y')
    ax2.set_title('Traiettoria Y')
    ax2.legend()
    ax2.grid()

    # Plot per z
    ax3 = fig.add_subplot(133)
    ax3.plot(z1, label='gnss 1')
    ax3.plot(z2, label='slam 2')
    ax3.set_xlabel('Indice')
    ax3.set_ylabel('Z')
    ax3.set_title('Traiettoria Z')
    ax3.legend()
    ax3.grid()

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
def icp_open3d(source, target, initial_transformation=np.eye(4), max_iterations=50):
    threshold = 0.2  # Soglia di distanza per ICP
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )
    return icp_result.transformation





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
interpolated_imu = interpolate_imu_with_PC(pointcloud_dir,imu_dict)






gnss_array, local_coordinates, reference_latitude, reference_longitude, scale_factor = plot_gnss_trajectory(gnss_file)



interp_lat, interp_lon, interp_alt = interpolate_gnss_data(gnss_array, local_coordinates)
pointcloud_files, transformation_matrices_original,dict_transformation = associate_pointclouds_with_gnss(pointcloud_dir, interp_lat, interp_lon,
                                                                            interp_alt, reference_latitude,
                                                                            reference_longitude, scale_factor)


interpolated_imu = normalize_transformation_dictionary(interpolated_imu)
plot_interpolated_matrices(interpolated_imu, dict_transformation)


# Ordinare i file in base al timestamp
pointcloud_files_sorted = sorted(pointcloud_files, key=get_timestamp)



pointclouds = []
matrixes = []
#interst: 1042-1125

start = 1042
for idx, file_name in enumerate(pointcloud_files_sorted):
    if start < idx < 1325:
        #SI PARTE DA i  = 1043
        #print(idx)
        timestamp_str = file_name.split('_')[1].split('.ply')[0]
        timestamp = float(timestamp_str)


        transformation_matrix = dict_transformation[str(timestamp)]

        pcd_raw = o3d.io.read_point_cloud(os.path.join(pointcloud_dir, file_name))  # Usa file_name invece di pointcloud_files[idx]
        pcd = remove_isolated_points(pcd_raw)
        pcd_m = convert_to_meters(pcd)
        pointclouds.append(pcd_m)
        matrixes.append(transformation_matrix)


#GNSS dall inizio del tracciato

normalized_transformations = normalize_matrix_trasformation(matrixes)

# per orientazione IMU
#normalize_transformation_list_rot_and_trasl()






absolute_transformations = [normalized_transformations[0]]  # Lista delle trasformazioni assolute
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
    #hybrid_absolute_trasform = matrix_mean_only_traslation(absolute_transformations[-1],)
    initial_transformation = absolute_transformations[-1]
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

absolute_positions = compute_absolute_positions(absolute_transformations)
print(absolute_positions[-1])

plot_trajectories(normalized_transformations, absolute_positions)