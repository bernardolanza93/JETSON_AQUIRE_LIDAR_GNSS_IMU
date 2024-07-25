from scipy.spatial.transform import Rotation as R
import numpy as np
import json



def compute_transformation_matrices_zed(imu_data, imu_timestamps):
    transformation_matrices = []
    integrated_angular_velocity = np.array([0.0, 0.0, 0.0])  # Initial orientation (Euler angles)
    last_time = imu_timestamps[0]

    for i, imu_msg in enumerate(imu_data):
        current_time = imu_timestamps[i]
        dt = current_time - last_time
        last_time = current_time

        # Extract angular velocity
        angular_velocity = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])

        # Integrate angular velocity to get Euler angles
        integrated_angular_velocity += angular_velocity * dt
        rotation_matrix = R.from_euler('xyz', integrated_angular_velocity).as_matrix()

        transformation_matrices.append(rotation_matrix)

    return transformation_matrices


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


def load_imu_data_from_json(json_file):
    #ONBLY FOR ORIENTATION (ERROR IN SAVE DATA)

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


def extract_dump_imu_orientation(bag_file, topic_gnss="/imu/data", output_file="imu_partial_xsens_data.json"):
    imu_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_gnss]):
            imu_entry = {
                'header': {
                    'seq': msg.header.seq,
                    'stamp': {
                        'secs': msg.header.stamp.secs,
                        'nsecs': msg.header.stamp.nsecs
                    },
                    'frame_id': msg.header.frame_id
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'orientation_covariance': list(msg.orientation_covariance),
                'timestamp': {
                    'secs': t.secs,
                    'nsecs': t.nsecs
                }
            }
            imu_data.append(imu_entry)

    with open(output_file, 'w') as f:
        json.dump(imu_data, f, indent=4)