import numpy as np
import json
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import pickle
import matplotlib.pyplot as plt


def filter_point_cloud(pcd, x_min=None, x_max=None, y_min=None, y_max=None, z_min=None, z_max=None):
    # Estrai punti e colori dalla nuvola di punti
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # Crea una maschera iniziale con tutte le condizioni True
    mask = np.ones(points.shape[0], dtype=bool)

    # Applica le soglie per ogni asse se specificate
    if x_min is not None:
        mask = mask & (points[:, 0] >= x_min)
    if x_max is not None:
        mask = mask & (points[:, 0] <= x_max)
    if y_min is not None:
        mask = mask & (points[:, 1] >= y_min)
    if y_max is not None:
        mask = mask & (points[:, 1] <= y_max)
    if z_min is not None:
        mask = mask & (points[:, 2] >= z_min)
    if z_max is not None:
        mask = mask & (points[:, 2] <= z_max)

    # Applica la maschera ai punti e ai colori
    filtered_points = points[mask]
    filtered_colors = colors[mask]

    # Crea la nuvola di punti filtrata
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    return filtered_pcd

def plot_pointcloud_translations(pre_icp_matrices, post_icp_matrices, third_icp_matrices=None):
    pre_translations = np.array([matrix[:2, 3] for matrix in pre_icp_matrices])
    pre_orientations = np.array([matrix[:2, 0] for matrix in pre_icp_matrices])

    post_translations = np.array([matrix[:2, 3] for matrix in post_icp_matrices])
    post_orientations = np.array([matrix[:2, 0] for matrix in post_icp_matrices])

    # Setup the plot
    plt.figure(figsize=(10, 8))

    # Plotting the pre-ICP translations and orientations
    plt.scatter(pre_translations[:, 0], pre_translations[:, 1], marker='o', color='g', label='GNSS-IMU', s=5)
    for i in range(len(pre_translations)):
        if i % 30 == 0:
            plt.arrow(pre_translations[i, 0], pre_translations[i, 1], pre_orientations[i, 0], pre_orientations[i, 1],
                      head_width=1, head_length=2, fc='g', ec='g')

    # Plotting the post-ICP translations and orientations
    plt.scatter(post_translations[:, 0], post_translations[:, 1], marker='x', color='r', label='SLAM-ICP', s=3)
    for i in range(len(post_translations)):
        if i % 30 == 0:
            plt.arrow(post_translations[i, 0], post_translations[i, 1], post_orientations[i, 0], post_orientations[i, 1],
                      head_width=0.7, head_length=1.5, fc='r', ec='r')

    # Plotting the third set of translations and orientations if provided
    if third_icp_matrices is not None:
        third_translations = np.array([matrix[:2, 3] for matrix in third_icp_matrices])
        third_orientations = np.array([matrix[:2, 0] for matrix in third_icp_matrices])
        plt.scatter(third_translations[:, 0], third_translations[:, 1], marker='^', color='b', label='Third Set', s=3)
        for i in range(len(third_translations)):
            if i % 30 == 0:
                plt.arrow(third_translations[i, 0], third_translations[i, 1], third_orientations[i, 0], third_orientations[i, 1],
                          head_width=0.4, head_length=1.1, fc='b', ec='b')

    # Final plot settings
    plt.title('Point Cloud Translations and Orientations in Local Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()



def extract_position_and_angles(transformation_matrix):
    # Estrai la posizione (traslazione) dai termini di traslazione della matrice
    position = transformation_matrix[:3, 3]
    x, y, z = position

    # Estrai la matrice di rotazione
    rotation_matrix = transformation_matrix[:3, :3]

    # Calcola gli angoli di rotazione (pitch, roll, yaw) dalla matrice di rotazione
    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = 0

    # Converti gli angoli da radiani a gradi
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    # Stampa i valori
    print(f"Position: x={x}, y={y}, z={z}")
    print(f"Angles: roll={roll_deg}°, pitch={pitch_deg}°, yaw={yaw_deg}°")

    return x, y, z, roll_deg, pitch_deg, yaw_deg

# Applichiamo queste matrici di trasformazione alle pointcloud fuse
# Applichiamo queste matrici di trasformazione alle pointcloud fuse
def apply_transformation(pointcloud, transformation):
    pointcloud.transform(transformation)
    return pointcloud



def save_pointcloud(pointcloud, filename):
    # Salva la PointCloud in un file
    o3d.io.write_point_cloud(filename, pointcloud)
    print(f"PointCloud salvata come {filename}")

def load_pointcloud(filename):
    # Carica la PointCloud da un file
    pointcloud = o3d.io.read_point_cloud(filename)
    print(f"PointCloud caricata da {filename}")
    return pointcloud
def adjust_rotation_matrix(rotation_matrix, axes_permutation):
    """
    Adjust the rotation matrix according to the specified axes permutation.
    :param rotation_matrix: Original rotation matrix (3x3)
    :param axes_permutation: Tuple indicating how to permute the axes (e.g., (1, 0, 2))
    :return: Adjusted rotation matrix
    """
    adjusted_matrix = rotation_matrix[:, axes_permutation]
    return adjusted_matrix
def compute_average_transformation(transformation_matrices, axes_permutation):
    # Estrazione delle rotazioni e delle traslazioni dalle matrici di trasformazione
    rotations = [matrix[:3, :3] for matrix in transformation_matrices]
    translations = [matrix[:3, 3] for matrix in transformation_matrices]

    # Aggiustamento delle matrici di rotazione
    adjusted_rotations = [adjust_rotation_matrix(rot, axes_permutation) for rot in rotations]

    # Calcolo della media delle rotazioni e delle traslazioni
    avg_rotation = np.mean(adjusted_rotations, axis=0)
    avg_translation = np.mean(translations, axis=0)

    # Ricostruzione della matrice di trasformazione media
    avg_transformation = np.eye(4)
    avg_transformation[:3, :3] = avg_rotation
    avg_transformation[:3, 3] = avg_translation

    return avg_transformation
def estrai_angoli_eulero(rot_matrix):
    if rot_matrix.shape != (3, 3):
        raise ValueError("La matrice di rotazione deve essere 3x3")

    # Estrazione degli angoli di Eulero
    if rot_matrix[2, 0] != 1 and rot_matrix[2, 0] != -1:
        theta_y = -np.arcsin(rot_matrix[2, 0])
        theta_x = np.arctan2(rot_matrix[2, 1] / np.cos(theta_y), rot_matrix[2, 2] / np.cos(theta_y))
        theta_z = np.arctan2(rot_matrix[1, 0] / np.cos(theta_y), rot_matrix[0, 0] / np.cos(theta_y))
    else:
        theta_z = 0
        if rot_matrix[2, 0] == -1:
            theta_y = np.pi / 2
            theta_x = theta_z + np.arctan2(rot_matrix[0, 1], rot_matrix[0, 2])
        else:
            theta_y = -np.pi / 2
            theta_x = -theta_z + np.arctan2(-rot_matrix[0, 1], -rot_matrix[0, 2])

    return theta_x, theta_y, theta_z


def matrice_rotazione_xyz(theta_x, theta_y, theta_z):
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x)],
        [0, np.sin(theta_x), np.cos(theta_x)]
    ])

    R_y = np.array([
        [np.cos(theta_y), 0, np.sin(theta_y)],
        [0, 1, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y)]
    ])

    R_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z), 0],
        [0, 0, 1]
    ])

    return R_z @ R_y @ R_x


def cambia_segno_angolo_z(rot_matrix):
    theta_x, theta_y, theta_z = estrai_angoli_eulero(rot_matrix)

    print(f"Angolo originale attorno all'asse z: {np.degrees(theta_z):.2f} gradi")

    # Cambia il segno dell'angolo attorno all'asse z
    theta_z_cambiato = -theta_z

    print(f"Angolo modificato attorno all'asse z: {np.degrees(theta_z_cambiato):.2f} gradi")

    # Ricostruisci la matrice di rotazione con l'angolo modificato
    rot_matrix_cambiata = matrice_rotazione_xyz(theta_x, theta_y, theta_z_cambiato)

    return rot_matrix_cambiata

def transform_point(point, transformation_matrix):
    """
    Trasforma un punto usando una matrice di trasformazione.

    :param point: Punto 3x1
    :param transformation_matrix: Matrice di trasformazione 4x4
    :return: Punto trasformato 3x1
    """
    point_homogeneous = np.append(point, 1)  # Converti in coordinate omogenee
    transformed_point_homogeneous = transformation_matrix @ point_homogeneous
    return transformed_point_homogeneous[:3]


def extract_positions(transformations):
    """
    Estrae le posizioni (x, y, z) da una lista di matrici di trasformazione.

    :param transformations: Lista di matrici di trasformazione 4x4
    :return: Tuple di liste (x_list, y_list, z_list)
    """
    x_list = []
    y_list = []
    z_list = []
    for T in transformations:
        _, translation = decompose_transformation_matrix(T)
        x_list.append(translation[0])
        y_list.append(translation[1])
        z_list.append(translation[2])
    return x_list, y_list, z_list

def compute_absolute_positions(transformations):
    """
    Calcola le posizioni assolute riferite al sistema di riferimento iniziale,
    con matrice di rotazione nulla (identica).

    :param transformations: Lista di matrici di trasformazione 4x4
    :return: Lista di matrici di trasformazione 4x4 con rotazione nulla
    """
    absolute_positions = []
    for T in transformations:
        R, t = decompose_transformation_matrix(T)
        T_inv = invert_transformation_matrix(R, t)
        absolute_translation = transform_point(np.zeros(3), T_inv)

        # Crea una nuova matrice di trasformazione con rotazione nulla
        T_abs = np.eye(4)
        T_abs[:3, 3] = absolute_translation
        absolute_positions.append(T_abs)

    return absolute_positions


def decompose_transformation_matrix(matrix):
    """
    Decomponi una matrice di trasformazione in rotazione e traslazione.

    :param matrix: Matrice di trasformazione 4x4
    :return: Tuple (rotazione 3x3, traslazione 3x1)
    """
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    return rotation, translation


def extract_euler_angles(rot_matrix):
    sy = np.sqrt(rot_matrix[0, 0] ** 2 + rot_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
        y = np.arctan2(-rot_matrix[2, 0], sy)
        z = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    else:
        x = np.arctan2(-rot_matrix[1, 2], rot_matrix[1, 1])
        y = np.arctan2(-rot_matrix[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def invert_transformation_matrix(rotation, translation):
    """
    Calcola la matrice di trasformazione inversa.

    :param rotation: Matrice di rotazione 3x3
    :param translation: Vettore di traslazione 3x1
    :return: Matrice di trasformazione inversa 4x4
    """
    rotation_inv = rotation.T
    translation_inv = -rotation_inv @ translation
    transformation_matrix_inv = np.eye(4)
    transformation_matrix_inv[:3, :3] = rotation_inv
    transformation_matrix_inv[:3, 3] = translation_inv
    return transformation_matrix_inv


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """


    w, x, y, z = Q
    # First row of the rotation matrix
    r00 = 1 - 2 * (y ** 2 + z ** 2)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    # Second row of the rotation matrix
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x ** 2 + z ** 2)
    r12 = 2 * (y * z - x * w)
    # Third row of the rotation matrix
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x ** 2 + y ** 2)
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def matrix_mean_only_traslation_old(matrice_rot_trasl,matrice_trasl):
    peso_rot_trasl =0.1
    peso_trasl = 0.9
    # Estrarre la traslazione dalle due matrici
    trasl_1 = matrice_rot_trasl[:3, 3]
    trasl_2 = matrice_trasl[:3, 3]



    # Calcolare la media delle traslazioni
    trasl_media_pesata = (peso_rot_trasl * trasl_1 + peso_trasl * trasl_2) / (peso_rot_trasl + peso_trasl)
    # Creare una nuova matrice di trasformazione mantenendo la rotazione e usando la traslazione media
    matrice_media = np.eye(4)
    matrice_media[:3, :3] = matrice_rot_trasl[:3, :3]  # Manteniamo la rotazione della prima matrice
    matrice_media[:3, 3] = trasl_media_pesata  # Usare la media delle traslazioni
    return matrice_media



def matrix_mean_with_rotation(matrice_rot_trasl, matrice_trasl, peso_rot_trasl=0.99999, peso_trasl=0.00001):
    # Estrarre la traslazione dalle due matrici
    #LA TRASLAZIONE NON È PIU VALIDA, NON CONOSCO GLI ASSI DEL GNSS E IL RAPPORTO CON L ASSE IDENTICO DELLA PRIMA POINTCLOUD, SO QUANTO SI SPOSTA MA NON SO IN CHE DIREZIONE
    trasl_1 = matrice_rot_trasl[:3, 3]
    trasl_2 = matrice_trasl[:3, 3] #gnss + IMU

    # Calcolare la media delle traslazioni
    #MEDIA TRA LE POSIZIONI ABILITARE QUI
    #trasl_media_pesata = (peso_rot_trasl * trasl_1 + peso_trasl * trasl_2) / (peso_rot_trasl + peso_trasl)
    #COSI DISABILITO MEDIA TRASLAZIONE
    trasl_media_pesata = matrice_rot_trasl[:3, 3]

    # Estrarre le matrici di rotazione
    rot_1 = matrice_rot_trasl[:3, :3]
    rot_2 = matrice_trasl[:3, :3]

    # Convertire le matrici di rotazione in quaternioni
    quat_1 = R.from_matrix(rot_1).as_quat()
    quat_2 = R.from_matrix(rot_2).as_quat()

    # Calcolare la media pesata dei quaternioni
    quat_media_pesata = (peso_rot_trasl * quat_1 + peso_trasl * quat_2) / (peso_rot_trasl + peso_trasl)
    quat_media_pesata /= np.linalg.norm(quat_media_pesata)  # Normalizzare il quaternione risultante

    # Convertire il quaternione medio risultante in una matrice di rotazione
    rot_media_pesata = R.from_quat(quat_media_pesata).as_matrix()

    # Creare una nuova matrice di trasformazione combinata
    matrice_media = np.eye(4)
    matrice_media[:3, :3] = rot_media_pesata  # Usare la rotazione media pesata
    matrice_media[:3, 3] = trasl_media_pesata  # Usare la media delle traslazioni

    return matrice_media


def normalize_transformation_list_rot_and_trasl(trans_list):
    # Prendere la prima matrice di trasformazione
    first_matrix = trans_list[0]
    first_rotation = first_matrix[:3, :3]
    first_translation = first_matrix[:3, 3]

    # Calcolare l'inversa della prima matrice di rotazione
    first_rotation_inv = np.linalg.inv(first_rotation)

    # Inizializzare la lista delle matrici normalizzate
    normalized_list = []
    cumulative_rotation = np.eye(3)  # Matrice di rotazione cumulativa

    for matrix in trans_list:
        rotation_matrix = matrix[:3, :3]
        translation_vector = matrix[:3, 3]

        # Normalizzare la matrice di rotazione
        normalized_rotation = first_rotation_inv @ rotation_matrix

        # Calcolare la rotazione incrementale rispetto alla rotazione precedente
        relative_rotation = cumulative_rotation.T @ rotation_matrix
        cumulative_rotation = rotation_matrix  # Aggiornare la rotazione cumulativa

        # Normalizzare il vettore di traslazione
        normalized_translation = first_rotation_inv @ (translation_vector - first_translation)

        # Creare la matrice di trasformazione normalizzata
        T_normalized = np.eye(4)
        T_normalized[:3, :3] = relative_rotation
        T_normalized[:3, 3] = normalized_translation

        normalized_list.append(T_normalized)

    return normalized_list

def normalize_transformation_dictionary(rot_dict, no_normalization = 1):
    print("NO NORMALIZATION IMU:",no_normalization)
    first_key = next(iter(rot_dict))
    first_matrix = rot_dict[first_key]
    first_rotation = first_matrix[:3, :3]

    # Calculate the inverse of the first rotation matrix
    first_rotation_inv = np.linalg.inv(first_rotation)

    normalized_dict = {}
    for timestamp, matrix in rot_dict.items():
        rotation_matrix = matrix[:3, :3]
        normalized_rotation = first_rotation_inv @ rotation_matrix

        # Create the normalized transformation matrix
        T_normalized = np.eye(4)
        if no_normalization:
            print("NO NORMALIZATION IMU:", no_normalization)
            T_normalized[:3, :3] = rotation_matrix
        else:
            T_normalized[:3, :3] = normalized_rotation
        T_normalized[:3, 3] = matrix[:3, 3]

        normalized_dict[timestamp] = T_normalized

    return normalized_dict


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



def extract_rotations(transformations):
    """
    Estrae le rotazioni (roll, pitch, yaw) da una lista di matrici di trasformazione.

    Args:
        transformations (list of np.ndarray): Lista di matrici di trasformazione.

    Returns:
        tuple of list: Tre liste contenenti le rotazioni roll, pitch, yaw.
    """
    roll = []
    pitch = []
    yaw = []

    for t in transformations:
        sy = np.sqrt(t[0, 0]**2 + t[1, 0]**2)

        singular = sy < 1e-6

        if not singular:
            yaw.append(np.arctan2(t[2, 1], t[2, 2]))
            pitch.append(np.arctan2(-t[2, 0], sy))
            roll.append(np.arctan2(t[1, 0], t[0, 0]))
        else:
            yaw.append(np.arctan2(-t[1, 2], t[1, 1]))
            pitch.append(np.arctan2(-t[2, 0], sy))
            roll.append(0)

    return roll, pitch, yaw
