import numpy as np
import json

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
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def matrix_mean_only_traslation(matrice_rot_trasl,matrice_trasl):
    # Estrarre la traslazione dalle due matrici
    trasl_1 = matrice_rot_trasl[:3, 3]
    trasl_2 = matrice_trasl[:3, 3]

    # Calcolare la media delle traslazioni
    trasl_media = (trasl_1 + trasl_2) / 2

    # Creare una nuova matrice di trasformazione mantenendo la rotazione e usando la traslazione media
    matrice_media = np.eye(4)
    matrice_media[:3, :3] = matrice_rot_trasl[:3, :3]  # Manteniamo la rotazione della prima matrice
    matrice_media[:3, 3] = trasl_media  # Usare la media delle traslazioni
    return matrice_media


def normalize_transformation_list_rot_and_trasl(trans_list):
    first_matrix = trans_list[0]
    first_rotation = first_matrix[:3, :3]
    first_translation = first_matrix[:3, 3]

    # Calculate the inverse of the first rotation matrix
    first_rotation_inv = np.linalg.inv(first_rotation)

    normalized_list = []
    for matrix in trans_list:
        rotation_matrix = matrix[:3, :3]
        translation_vector = matrix[:3, 3]

        # Normalize the rotation matrix
        normalized_rotation = first_rotation_inv @ rotation_matrix

        # Normalize the translation vector
        normalized_translation = first_rotation_inv @ (translation_vector - first_translation)

        # Create the normalized transformation matrix
        T_normalized = np.eye(4)
        T_normalized[:3, :3] = normalized_rotation
        T_normalized[:3, 3] = normalized_translation

        normalized_list.append(T_normalized)

    return normalized_list

def normalize_transformation_dictionary(rot_dict):
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