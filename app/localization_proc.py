import json
import sys

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import datetime
from matplotlib.ticker import MaxNLocator

from skinematics import quat
from skinematics.quat import Quaternion
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick
from ahrs.common.orientation import q_rot
import plotter as PLOT


def moving_average(data, window_size):
    """
    Apply a moving average filter to 1D data with specified window size.
    Returns data trimmed to match the length of the filtered result.
    """
    filtered_data = np.convolve(data, np.ones(window_size) / window_size, mode='valid')
    return filtered_data
def moving_average(data, window_size):
    """
    Apply a moving average filter to 1D data with specified window size.
    """
    return np.convolve(data, np.ones(window_size) / window_size, mode='same')
def generate_gnss_based_orientation(gnss_data_processed, position_window=5, orientation_window=5):
    """
    Generate arbitrary orientation frames from GNSS trajectory data with moving average filtering.

    Parameters:
        gnss_data_processed (dict): Processed GNSS data with timestamp keys and position values.
        position_window (int): Window size for the moving average filter on positions.
        orientation_window (int): Window size for the moving average filter on orientations.

    Returns:
        interpolated_data (dict): Dictionary with filtered timestamps as keys and associated rotation and position values.
    """
    # Extract GNSS timestamps and positions from the processed data
    gnss_timestamps = []
    gnss_positions = []

    for timestamp, data in gnss_data_processed.items():
        gnss_timestamps.append(float(timestamp))
        position = data['position']
        gnss_positions.append([position['x'], position['y'], position['z']])

    # Convert inputs to numpy arrays for easier processing
    gnss_timestamps = np.array(gnss_timestamps)
    gnss_positions = np.array(gnss_positions)

    # Apply moving average filter to positions
    filtered_positions = np.array([moving_average(gnss_positions[:, i], position_window) for i in range(3)]).T
    filtered_timestamps = gnss_timestamps[position_window - 1 : -position_window + 1] if position_window > 1 else gnss_timestamps


    # print(len(filtered_positions[:, 1]))
    # print(len(filtered_timestamps))


    # Initialize lists for orientations
    roll_list, pitch_list, yaw_list = [], [], []

    for i in range(1, len(filtered_positions)):
        # Calcolo dell'asse x come vettore di spostamento
        tangent_vector = filtered_positions[i] - filtered_positions[i - 1]
        x_axis = tangent_vector / np.linalg.norm(tangent_vector)

        # Calcolo degli assi y e z come ortogonali a x
        y_axis = np.cross([0, 0, 1], x_axis)
        if np.linalg.norm(y_axis) < 1e-6:
            y_axis = np.cross([0, 1, 0], x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)

        # Calcolo degli angoli
        pitch = np.arcsin(-x_axis[2])
        roll = np.arctan2(z_axis[1], z_axis[2])
        yaw = np.arctan2(y_axis[0], x_axis[0])

        roll_list.append(roll)
        pitch_list.append(pitch)
        yaw_list.append(yaw)

    roll_list = np.unwrap(roll_list)
    pitch_list = np.unwrap(pitch_list)
    yaw_list = np.unwrap(yaw_list)






    # Filtra le rotazioni e regola la lunghezza dei timestamp
    filtered_roll = moving_average(roll_list, orientation_window)
    filtered_pitch = moving_average(pitch_list, orientation_window)
    filtered_yaw = moving_average(yaw_list, orientation_window)

    # Allinea i timestamp e le posizioni con la lunghezza degli angoli filtrati
    min_length = min(len(filtered_roll), len(filtered_pitch), len(filtered_yaw), len(filtered_timestamps), len(filtered_positions))

    final_timestamps = filtered_timestamps[:min_length]
    final_positions = filtered_positions[:min_length]
    filtered_roll = filtered_roll[:min_length]
    filtered_pitch = filtered_pitch[:min_length]
    filtered_yaw = filtered_yaw[:min_length]




    # Costruzione del dizionario di output con i dati filtrati
    interpolated_data = {}
    for i, timestamp in enumerate(final_timestamps):
        interpolated_data[timestamp] = {
            'rotation': {
                'x': filtered_roll[i],
                'y': filtered_pitch[i],
                'z': filtered_yaw[i]
            },
            'position': {
                'x': final_positions[i, 0],
                'y': final_positions[i, 1],
                'z': final_positions[i, 2]
            }
        }

    return interpolated_data



def normalize_angle(angle):
    """ Normalizza un angolo in radianti tra -π e π """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def resample_data_to_slam_frequency(slam_timestamps, interpolated_data):
    """
    Resample IMU + GNSS interpolated data to match SLAM timestamps and normalize data to start from zero.

    Parameters:
        slam_timestamps (list): List of SLAM timestamps.
        interpolated_data (dict): Dictionary containing IMU rotations and GNSS positions with timestamps as keys.

    Returns:
        resampled_data (dict): Resampled IMU + GNSS data to match SLAM timestamps, normalized to start from zero.
    """
    # Sort SLAM timestamps
    slam_timestamps = sorted(slam_timestamps)

    # Extract IMU + GNSS timestamps and values
    imu_gnss_timestamps = np.array(sorted(interpolated_data.keys()))

    # Convert GNSS timestamps to nanoseconds (from seconds) to match SLAM format
    imu_gnss_timestamps = imu_gnss_timestamps * 1e9  # Convert seconds to nanoseconds

    # Calculate frequencies after timestamp alignment
    imu_gnss_frequency = 1 / np.mean(np.diff(imu_gnss_timestamps)) * 1e9
    slam_frequency = 1 / np.mean(np.diff(slam_timestamps)) * 1e9


    # Extract the positions and rotations
    imu_gnss_positions = np.array([[
        interpolated_data[t]['position']['x'],
        interpolated_data[t]['position']['y'],
        interpolated_data[t]['position']['z']] for t in interpolated_data.keys()])
    imu_gnss_rotations = np.array([[
        interpolated_data[t]['rotation']['x'],
        interpolated_data[t]['rotation']['y'],
        interpolated_data[t]['rotation']['z']] for t in interpolated_data.keys()])



    imu_gnss_positions_transformed = []
    for pos in imu_gnss_positions:
        transformed_pos = [
            -pos[2],
            pos[0],
            -pos[1]
        ]
        imu_gnss_positions_transformed.append(transformed_pos)
    imu_gnss_positions = np.array(imu_gnss_positions_transformed)


    imu_gnss_rotations_transformed = []
    for rot in imu_gnss_rotations:
        transformed_rot = [
            -rot[2],
            rot[0],
            -rot[1]
        ]
        imu_gnss_rotations_transformed.append(transformed_rot)
    imu_gnss_rotations = np.array(imu_gnss_rotations_transformed)


    print("GNSS FREQ",imu_gnss_frequency)
    print("SLAM FREQ", slam_frequency)

    # Set up resampling to match SLAM timestamps
    if imu_gnss_frequency < slam_frequency:
        print("ONLY GNSS!!!")
        # Upsample GNSS/IMU data to SLAM frequency
        interp_func_x = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 0], kind='linear', fill_value="extrapolate")
        interp_func_y = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 1], kind='linear', fill_value="extrapolate")
        interp_func_z = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 2], kind='linear', fill_value="extrapolate")

        interp_func_roll = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 0], kind='linear',
                                    fill_value="extrapolate")
        interp_func_pitch = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 1], kind='linear',
                                     fill_value="extrapolate")
        interp_func_yaw = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 2], kind='linear',
                                   fill_value="extrapolate")
    else:
        # Downsample GNSS/IMU data to SLAM frequency
        interp_func_x = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 0], kind='nearest',
                                 fill_value="extrapolate")
        interp_func_y = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 1], kind='nearest',
                                 fill_value="extrapolate")
        interp_func_z = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 2], kind='nearest',
                                 fill_value="extrapolate")

        interp_func_roll = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 0], kind='nearest',
                                    fill_value="extrapolate")
        interp_func_pitch = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 1], kind='nearest',
                                     fill_value="extrapolate")
        interp_func_yaw = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 2], kind='nearest',
                                   fill_value="extrapolate")

    # Build the output dictionary
    resampled_data = {}
    for t in slam_timestamps:
        resampled_data[t] = {
            'rotation': {
                'x': interp_func_roll(t),
                'y': interp_func_pitch(t),
                'z': interp_func_yaw(t)
            },
            'position': {
                'x': interp_func_x(t),
                'y': interp_func_y(t),
                'z': interp_func_z(t)
            }
        }

    # Normalize positions and rotations
    first_position = np.array([resampled_data[slam_timestamps[0]]['position']['x'],
                               resampled_data[slam_timestamps[0]]['position']['y'],
                               resampled_data[slam_timestamps[0]]['position']['z']])
    first_rotation = np.array([resampled_data[slam_timestamps[0]]['rotation']['x'],
                               resampled_data[slam_timestamps[0]]['rotation']['y'],
                               resampled_data[slam_timestamps[0]]['rotation']['z']])

    for t in slam_timestamps:
        resampled_data[t]['position']['x'] -= first_position[0]
        resampled_data[t]['position']['y'] -= first_position[1]
        resampled_data[t]['position']['z'] -= first_position[2]
        resampled_data[t]['rotation']['x'] -= first_rotation[0]
        resampled_data[t]['rotation']['y'] -= first_rotation[1]
        resampled_data[t]['rotation']['z'] -= first_rotation[2]

    return resampled_data


def downsample_interpolated_data_to_slam(slam_timestamps, interpolated_data):
    """
    Downsample IMU + GNSS interpolated data to match SLAM timestamps and normalize the data to start from zero.

    Parameters:
        slam_timestamps (list): List of SLAM timestamps.
        interpolated_data (dict): Dictionary containing IMU rotations and GNSS positions with timestamps as keys.

    Returns:
        downsampled_data (dict): Interpolated IMU + GNSS data to match SLAM timestamps, normalized to start from zero.
    """
    # Sort SLAM timestamps
    slam_timestamps = sorted(slam_timestamps)

    # Extract IMU + GNSS timestamps and values
    imu_gnss_timestamps = np.array(sorted(interpolated_data.keys()))

    # Convert GNSS timestamps to match SLAM format by adding an appropriate offset
    # Assuming GNSS timestamps are in seconds, convert them to match SLAM timestamps (which are in nanoseconds)
    imu_gnss_timestamps = imu_gnss_timestamps * 1e9  # Convert seconds to nanoseconds

    # Extract the positions and rotations
    imu_gnss_positions = np.array([[
        interpolated_data[t]['position']['x'],
        interpolated_data[t]['position']['y'],
        interpolated_data[t]['position']['z']] for t in interpolated_data.keys()])
    imu_gnss_rotations = np.array([[
        interpolated_data[t]['rotation']['x'],
        interpolated_data[t]['rotation']['y'],
        interpolated_data[t]['rotation']['z']] for t in interpolated_data.keys()])

    # Inversione degli assi per rendere il sistema GNSS destrorso
    imu_gnss_positions[:, 0] *= 1  # Inverti X
    imu_gnss_positions[:, 1] *= 1  # Inverti Y
    imu_gnss_rotations[:, 0] *= -1  # Inverti rotazione intorno a X
    imu_gnss_rotations[:, 1] *= -1  # Inverti rotazione intorno a Y

    # Rotazione degli assi per allineare il sistema GNSS/IMU al sistema SLAM
    # X (GNSS) -> Y (SLAM)
    # Y (GNSS) -> -Z (SLAM)
    # Z (GNSS) -> -X (SLAM)
    rotation_matrix = np.array([
        [0, 0, -1],  # X (SLAM) = -Z (GNSS)
        [1, 0, 0],  # Y (SLAM) = X (GNSS)
        [0, -1, 0]  # Z (SLAM) = -Y (GNSS)
    ])

    # Applicare la trasformazione alle posizioni
    imu_gnss_positions = np.dot(imu_gnss_positions, rotation_matrix.T)

    # Calcolo delle rotazioni modificate come rotazioni nello spazio
    imu_gnss_rotations_transformed = []
    for rot in imu_gnss_rotations:
        # Scambiare i valori degli angoli per allineare con il nuovo sistema di riferimento
        transformed_rot = [
            rot[2],  # L'angolo di rotazione Z del GNSS diventa X del SLAM
            rot[0],  # L'angolo di rotazione X del GNSS diventa Y del SLAM
            -rot[1]  # L'angolo di rotazione Y del GNSS diventa -Z del SLAM
        ]

        imu_gnss_rotations_transformed.append(transformed_rot)

    imu_gnss_rotations = np.array(imu_gnss_rotations_transformed)



    # Determine the common time range
    start_time = max(slam_timestamps[0], imu_gnss_timestamps[0])
    end_time = min(slam_timestamps[-1], imu_gnss_timestamps[-1])
    valid_slam_timestamps = [t for t in slam_timestamps if start_time <= t <= end_time]

    # Interpolate IMU + GNSS positions to match SLAM timestamps
    interp_func_x = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 0], kind='linear', fill_value="extrapolate")
    interp_func_y = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 1], kind='linear', fill_value="extrapolate")
    interp_func_z = interp1d(imu_gnss_timestamps, imu_gnss_positions[:, 2], kind='linear', fill_value="extrapolate")

    # Interpolate IMU rotations to match SLAM timestamps
    interp_func_roll = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 0], kind='linear', fill_value="extrapolate")
    interp_func_pitch = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 1], kind='linear', fill_value="extrapolate")
    interp_func_yaw = interp1d(imu_gnss_timestamps, imu_gnss_rotations[:, 2], kind='linear', fill_value="extrapolate")

    # Create the downsampled dictionary
    downsampled_data = {}
    for t in valid_slam_timestamps:
        downsampled_data[t] = {
            'rotation': {
                'x': interp_func_roll(t),
                'y': interp_func_pitch(t),
                'z': interp_func_yaw(t)
            },
            'position': {
                'x': interp_func_x(t),
                'y': interp_func_y(t),
                'z': interp_func_z(t)
            }
        }

    # Normalize positions and rotations to start from zero after interpolation
    first_position = np.array([downsampled_data[valid_slam_timestamps[0]]['position']['x'],
                               downsampled_data[valid_slam_timestamps[0]]['position']['y'],
                               downsampled_data[valid_slam_timestamps[0]]['position']['z']])
    first_rotation = np.array([downsampled_data[valid_slam_timestamps[0]]['rotation']['x'],
                               downsampled_data[valid_slam_timestamps[0]]['rotation']['y'],
                               downsampled_data[valid_slam_timestamps[0]]['rotation']['z']])

    for t in valid_slam_timestamps:
        downsampled_data[t]['position']['x'] -= first_position[0]
        downsampled_data[t]['position']['y'] -= first_position[1]
        downsampled_data[t]['position']['z'] -= first_position[2]
        downsampled_data[t]['rotation']['x'] -= first_rotation[0]
        downsampled_data[t]['rotation']['y'] -= first_rotation[1]
        downsampled_data[t]['rotation']['z'] -= first_rotation[2]

    return downsampled_data
def align_gnss_trajectory(gnss_data_processed):
    """
    Align GNSS trajectory so that the initial motion is along the X direction (1, 0, 0),
    by applying a rotation around the Z axis only.

    Parameters:
        gnss_data_processed (dict): Processed GNSS data with timestamp keys and position values.

    Returns:
        aligned_gnss_data_processed (dict): GNSS data after alignment.
    """
    # Extract GNSS timestamps and positions from the processed data
    gnss_timestamps = []
    gnss_positions = []

    for timestamp, data in gnss_data_processed.items():
        gnss_timestamps.append(float(timestamp))
        position = data['position']
        gnss_positions.append([position['x'], position['y'], position['z']])

    gnss_positions = np.array(gnss_positions)

    # Calculate the average initial direction of motion (using the first few points)
    N = 400  # Number of initial points to estimate the direction
    initial_vector = gnss_positions[N] - gnss_positions[0]  # Difference between the first N points
    initial_direction_xy = initial_vector[:2]  # Consider only the XY plane
    initial_direction_xy = initial_direction_xy / np.linalg.norm(initial_direction_xy)

    # Calculate the rotation needed to align the initial direction with [1, 0] in the XY plane
    target_direction = np.array([1, 0])
    angle = np.arctan2(initial_direction_xy[1], initial_direction_xy[0]) - np.arctan2(target_direction[1], target_direction[0])

    # Create a rotation matrix around the Z axis
    rotation_matrix = np.array([
        [np.cos(-angle), -np.sin(-angle), 0],
        [np.sin(-angle), np.cos(-angle), 0],
        [0, 0, 1]
    ])

    # Rotate all positions around the Z axis
    gnss_positions_aligned = (rotation_matrix @ gnss_positions.T).T

    # Create a new dictionary with the aligned positions
    aligned_gnss_data_processed = {}
    for i, timestamp in enumerate(gnss_timestamps):
        aligned_gnss_data_processed[timestamp] = {
            'position': {
                'x': gnss_positions_aligned[i, 0],
                'y': gnss_positions_aligned[i, 1],
                'z': gnss_positions_aligned[i, 2]
            }
        }

    return aligned_gnss_data_processed





def interpolate_imu_gnss(imu_timestamps, imu_rotations, gnss_data_processed):
    """
    Interpolate GNSS positions to match IMU timestamps and associate IMU rotations.

    Parameters:
        imu_timestamps (array): Timestamps of IMU data.
        imu_rotations (array): Rotations from IMU data (in degrees).
        gnss_data_processed (dict): Processed GNSS data with timestamp keys and position values.

    Returns:
        interpolated_data (dict): Dictionary with timestamps as keys and associated rotation and position values.
    """
    # Extract GNSS timestamps and positions from the processed data
    gnss_timestamps = []
    gnss_positions = []

    for timestamp, data in gnss_data_processed.items():
        gnss_timestamps.append(float(timestamp))
        position = data['position']
        gnss_positions.append([position['x'], position['y'], position['z']])

    # Convert inputs to numpy arrays for easier processing
    imu_timestamps = np.array(imu_timestamps)
    imu_rotations = np.radians(np.array(imu_rotations))  # Convert rotations to radians
    gnss_timestamps = np.array(gnss_timestamps)
    gnss_positions = np.array(gnss_positions)

    # Create an interpolation function for each axis of the GNSS positions
    interp_func_x = interp1d(gnss_timestamps, gnss_positions[:, 0], kind='linear', fill_value="extrapolate")
    interp_func_y = interp1d(gnss_timestamps, gnss_positions[:, 1], kind='linear', fill_value="extrapolate")
    interp_func_z = interp1d(gnss_timestamps, gnss_positions[:, 2], kind='linear', fill_value="extrapolate")

    # Interpolate GNSS positions to match the IMU timestamps
    interpolated_x = interp_func_x(imu_timestamps)
    interpolated_y = interp_func_y(imu_timestamps)
    interpolated_z = interp_func_z(imu_timestamps)

    # Stack the interpolated values to create a position array
    interpolated_positions = np.vstack((interpolated_x, interpolated_y, interpolated_z)).T

    # Normalize the rotations to start from zero
    initial_rotation = R.from_euler('xyz', imu_rotations[0])
    adjusted_rotations = []

    for rotation in imu_rotations:
        current_rotation = R.from_euler('xyz', rotation)
        relative_rotation = current_rotation * initial_rotation.inv()
        adjusted_rotations.append(relative_rotation.as_euler('xyz'))

    adjusted_rotations = np.array(adjusted_rotations)  # Keep rotations in radians


    # Create output dictionary
    interpolated_data = {}
    for i, timestamp in enumerate(imu_timestamps):
        interpolated_data[timestamp] = {
            'rotation': {
                'x': adjusted_rotations[i, 0],
                'y': adjusted_rotations[i, 1],
                'z': adjusted_rotations[i, 2]
            },
            'position': {
                'x': interpolated_positions[i, 0],
                'y': interpolated_positions[i, 1],
                'z': interpolated_positions[i, 2]
            }
        }

    return interpolated_data

def rotate_gravity(gravity_vector, delta_rotation_matrix):
    """
    Ruota il vettore di gravità usando la matrice di rotazione relativa.

    gravity_vector: Vettore di gravità da ruotare.
    delta_rotation_matrix: Matrice di rotazione relativa da applicare.
    """
    # Ruota la gravità usando la matrice di rotazione relativa
    return np.dot(delta_rotation_matrix.T, gravity_vector)

def timestamp_to_hms(timestamps):
    """
    Converte una lista di timestamp in formato HH:MM:SS.mmm, dove mmm sono i millisecondi.
    """
    return [datetime.datetime.utcfromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3] for ts in timestamps]

def calculate_frequencies(timestamps):
    # Calcola la differenza tra i timestamp consecutivi (delta t)
    delta_t = np.diff(timestamps)

    # Calcola la frequenza media come l'inverso della media di delta t
    average_delta_t = np.mean(delta_t)
    average_frequency = 1.0 / average_delta_t

    return average_frequency
def quaternion_to_euler(q):
    """Convert a quaternion into Euler angles (roll, pitch, yaw)."""
    r = R.from_quat([q['x'], q['y'], q['z'], q['w']])
    return r.as_euler('xyz', degrees=False)  # return in radians


def plot_trajectory_3d_imuu(timestamps, trajectory, rotations):
    trajectory = np.array(trajectory)
    rotations = np.array(rotations)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the entire trajectory
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o', label='Trajectory')

    # Highlight the first 10 points with different colors
    for i in range(10):
        ax.scatter(trajectory[i, 0], trajectory[i, 1], trajectory[i, 2], color=plt.cm.jet(i / 10), s=50, label=f'Point {i+1}' if i == 0 else "")

    # Plot an arrow every 100 points based on rotations
    arrow_length = 5.0  # Increased length of the arrow for better visualization
    for i in range(0, len(trajectory), 200):
        position = trajectory[i]
        rotation = R.from_euler('xyz', rotations[i], degrees=True)  # Assuming rotations are in Euler angles
        direction = rotation.apply([1, 0, 0])  # Assuming the arrow points along the x-axis in the local frame
        ax.quiver(position[0], position[1], position[2],
                  direction[0], direction[1], direction[2],
                  length=arrow_length, color='r')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory with Orientation Arrows')

    # Setting equal scaling for all axes
    ax.set_box_aspect([np.ptp(trajectory[:, 0]), np.ptp(trajectory[:, 1]), np.ptp(trajectory[:, 2])])

    ax.legend()
    plt.show()




def plot_rotations(timestamps, rotations):
    rotations = np.array(rotations)

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax[0].plot(timestamps, rotations[:, 0], label='Roll')
    ax[0].set_ylabel('Roll (degrees)')
    ax[0].legend()

    ax[1].plot(timestamps, rotations[:, 1], label='Pitch')
    ax[1].set_ylabel('Pitch (degrees)')
    ax[1].legend()

    ax[2].plot(timestamps, rotations[:, 2], label='Yaw')
    ax[2].set_ylabel('Yaw (degrees)')
    ax[2].set_xlabel('Timestamp (s)')
    ax[2].legend()

    plt.tight_layout()
    plt.show()



def moving_average(data, window_size):
    """Apply a moving average filter to the data."""
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def plot_accelerations(timestamps, accelerations, window_size=600):
    accelerations = np.array(accelerations)
    filtered_accelerations = np.zeros((accelerations.shape[0] - window_size + 1, accelerations.shape[1]))

    for i in range(accelerations.shape[1]):
        filtered_accelerations[:, i] = moving_average(accelerations[:, i], window_size)

    filtered_timestamps = timestamps[:filtered_accelerations.shape[0]]

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax[0].plot(filtered_timestamps, filtered_accelerations[:, 0], label='X Acceleration (filtered)')
    ax[0].set_ylabel('X Acceleration (m/s^2)')
    ax[0].legend()

    ax[1].plot(filtered_timestamps, filtered_accelerations[:, 1], label='Y Acceleration (filtered)')
    ax[1].set_ylabel('Y Acceleration (m/s^2)')
    ax[1].legend()

    ax[2].plot(filtered_timestamps, filtered_accelerations[:, 2], label='Z Acceleration (filtered)')
    ax[2].set_ylabel('Z Acceleration (m/s^2)')
    ax[2].set_xlabel('Timestamp (s)')
    ax[2].legend()

    plt.tight_layout()
    plt.show()


def plot_translations_double_confront(timestamps_gnss, trajectory_gnss, timestamps_imu, trajectory_imu, label="GNSS vs IMU"):
    """
    Plotta le traiettorie X, Y, Z per GNSS e IMU su 3 subplot con timestamp condiviso.
    """
    # Converti le traiettorie in array NumPy
    trajectory_gnss = np.array(trajectory_gnss)
    trajectory_imu = np.array(trajectory_imu)

    # Converte i timestamp in formato HH:MM:SS.mmm per GNSS e IMU
    # time_labels_gnss = timestamp_to_hms(timestamps_gnss)
    # time_labels_imu = timestamp_to_hms(timestamps_imu)
    time_labels_gnss = timestamps_gnss
    time_labels_imu = timestamps_imu

    # Crea i subplot
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f'Spostamenti X,Y,Z sensore: {label}', fontsize=16)

    # Subplot X
    ax[0].plot(time_labels_gnss, trajectory_gnss[:, 0], label='GNSS X Position', color='blue')
    ax[0].plot(time_labels_imu, trajectory_imu[:, 0] / 1000, label='IMU X Position', color='orange')
    ax[0].set_ylabel('X Position (m)')
    ax[0].legend()

    # Subplot Y
    ax[1].plot(time_labels_gnss, trajectory_gnss[:, 1], label='GNSS Y Position', color='blue')
    ax[1].plot(time_labels_imu, trajectory_imu[:, 1] / 1000, label='IMU Y Position', color='orange')
    ax[1].set_ylabel('Y Position (m)')
    ax[1].legend()

    # Subplot Z
    ax[2].plot(time_labels_gnss, trajectory_gnss[:, 2], label='GNSS Z Position', color='blue')
    ax[2].plot(time_labels_imu, trajectory_imu[:, 2] / 1000, label='IMU Z Position', color='orange')
    ax[2].set_ylabel('Z Position (m)')
    ax[2].set_xlabel('Timestamp (HH:MM:SS.mmm)')
    ax[2].legend()

    # Usa MaxNLocator per limitare il numero di tick visibili sull'asse X
    for axis in ax:
        axis.xaxis.set_major_locator(MaxNLocator(nbins=10))

    # Ruota le etichette per renderle leggibili
    plt.setp(ax[2].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

    # Spaziatura ottimale
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

def plot_accelerations_and_rotations(timestamps, velocities, accelerations, rotations, label="no data"):
    accelerations = np.array(accelerations)
    velocities = np.array(velocities)
    rotations = np.array(rotations)

    # Converti i timestamp in etichette leggibili
    time_labels = timestamp_to_hms(timestamps)

    # Crea una figura con 9 subplot
    fig, ax = plt.subplots(9, 1, figsize=(12, 18), sharex=True)
    fig.suptitle(f'Spostamenti X, Y, Z sensore: {label}', fontsize=16)  # Titolo generale

    # Plottare velocità
    ax[0].plot(time_labels, velocities[:, 0], label='X Velocity')
    ax[0].set_ylabel('X Velocity (m/s)')
    ax[0].legend()

    ax[1].plot(time_labels, velocities[:, 1], label='Y Velocity')
    ax[1].set_ylabel('Y Velocity (m/s)')
    ax[1].legend()

    ax[2].plot(time_labels, velocities[:, 2], label='Z Velocity')
    ax[2].set_ylabel('Z Velocity (m/s)')
    ax[2].legend()

    # Plottare accelerazioni
    ax[3].plot(time_labels, accelerations[:, 0], label='X Acceleration')
    ax[3].set_ylabel('X Acceleration (m/s²)')
    ax[3].legend()

    ax[4].plot(time_labels, accelerations[:, 1], label='Y Acceleration')
    ax[4].set_ylabel('Y Acceleration (m/s²)')
    ax[4].legend()

    ax[5].plot(time_labels, accelerations[:, 2], label='Z Acceleration')
    ax[5].set_ylabel('Z Acceleration (m/s²)')
    ax[5].legend()

    # Plottare rotazioni
    ax[6].plot(time_labels, rotations[:, 0], label='X Rotation')
    ax[6].set_ylabel('X Rotation (rad)')
    ax[6].legend()

    ax[7].plot(time_labels, rotations[:, 1], label='Y Rotation')
    ax[7].set_ylabel('Y Rotation (rad)')
    ax[7].legend()

    ax[8].plot(time_labels, rotations[:, 2], label='Z Rotation')
    ax[8].set_ylabel('Z Rotation (rad)')
    ax[8].set_xlabel('Timestamp (s)')
    ax[8].legend()

    # Limita il numero di tick sull'asse X
    for axis in ax:
        axis.xaxis.set_major_locator(MaxNLocator(nbins=10))

    # Ruota le etichette dell'asse X per renderle leggibili
    plt.setp(ax[8].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")

    plt.tight_layout()
    plt.show()


def plot_translations(timestamps, trajectory, label = "non specificato"):
    trajectory = np.array(trajectory)



    time_labels = timestamp_to_hms(timestamps)

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f'Spostamenti X,Y,Z sensore: {label}', fontsize=16)  # Titolo generale

    ax[0].plot(time_labels, trajectory[:, 0], label='X Position')
    ax[0].set_ylabel('X Position (m)')
    ax[0].legend()

    ax[1].plot(time_labels, trajectory[:, 1], label='Y Position')
    ax[1].set_ylabel('Y Position (m)')
    ax[1].legend()

    ax[2].plot(time_labels, trajectory[:, 2], label='Z Position')
    ax[2].set_ylabel('Z Position (m)')
    ax[2].set_xlabel('Timestamp (s)')
    ax[2].legend()

    # Usa MaxNLocator per limitare il numero di tick visibili sull'asse X
    for axis in ax:
        axis.xaxis.set_major_locator(MaxNLocator(nbins=10))  # Mostra al massimo 10 etichette sull'asse X

    # Ruota le etichette per renderle leggibili
    plt.setp(ax[2].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")


    plt.tight_layout()
    plt.show()


def scykit_process_imu(file_path):


    with open(file_path) as f:
        imu_data = json.load(f)

    timestamps = []
    accelerations = []
    angular_velocities = []
    orientations = []


    for timestamp, data in sorted(imu_data.items()):
        timestamps.append(float(timestamp))

        orientations.append([
            data['orientation']['w'],
            data['orientation']['x'],
            data['orientation']['y'],
            data['orientation']['z']
        ])  # scikit-kinematics utilizza l'ordine (w, x, y, z)

        accelerations.append([
            data['linear_acceleration']['x'],
            data['linear_acceleration']['y'],
            data['linear_acceleration']['z']
        ])

        angular_velocities.append([
            data['angular_velocity']['x'],
            data['angular_velocity']['y'],
            data['angular_velocity']['z']
        ])

    # Converti le liste in array NumPy
    timestamps = np.array(timestamps)
    accelerations = np.array(accelerations)
    angular_velocities = np.array(angular_velocities)
    orientations = np.array(orientations)

    #angular_velocities = np.deg2rad(angular_velocities)

    # Calculate time intervals
    dt = np.diff(timestamps, prepend=timestamps[0])

    # Assicurati che le unità siano corrette
    # Se necessario, converte le velocità angolari in rad/s
    # angular_velocities = np.deg2rad(angular_velocities)

    # Frequenza di campionamento
    fs = 1 / np.mean(dt)  # Hz

    # Inizializza l'algoritmo Madgwick
    madgwick = Madgwick(sampleperiod=1 / fs)

    # Inizializza array per i quaternioni
    num_samples = len(timestamps)
    quaternions = np.zeros((num_samples, 4))

    # Quaternione iniziale
    q = np.array([1.0, 0.0, 0.0, 0.0])

    # Stima dei quaternioni
    for i in range(num_samples):
        q = madgwick.updateIMU(q, angular_velocities[i], accelerations[i])
        quaternions[i] = q

    # Rotazione delle accelerazioni nel sistema globale
    acc_global = np.zeros_like(accelerations)
    for i in range(len(accelerations)):
        acc_global[i] = q_rot(quaternions[i], accelerations[i])

    # Sottrai la gravità
    gravity = np.array([0, 0, 9.81])  # m/s^2
    acc_global -= gravity

    # Integrazione per ottenere le velocità
    velocities = np.zeros_like(acc_global)
    for i in range(1, len(acc_global)):
        velocities[i] = velocities[i - 1] + acc_global[i] * dt[i]

    # Integrazione per ottenere le posizioni
    positions = np.zeros_like(velocities)
    for i in range(1, len(velocities)):
        positions[i] = positions[i - 1] + velocities[i] * dt[i]

    # Visualizza la traiettoria
    plt.figure(figsize=(10, 6))
    plt.plot(positions[:, 0], positions[:, 1], label='Traiettoria XY')
    plt.xlabel('Posizione X (m)')
    plt.ylabel('Posizione Y (m)')
    plt.title('Traiettoria Ricostruita con AHRS')
    plt.legend()
    plt.grid(True)
    plt.show()

    sys.exit()



def process_imu_data(file_path, euler_order='xyz', degrees = True):



    #yxz
    def euler_to_rotation_matrix(angles, order):
        """Convert Euler angles to a rotation matrix with specified order."""
        return R.from_euler(order, angles, degrees).as_matrix()
    def transform_acceleration(acc, rotation_matrix):
        """Transform acceleration from IMU frame to global frame."""
        return np.dot(rotation_matrix, acc)

    def quaternion_to_euler(q, order, degrees_i = degrees):
        """Convert a quaternion into Euler angles with the specified order."""
        r = R.from_quat(q)
        return r.as_euler(order, degrees=degrees_i)


    def quaternion_rotate_vector(vector, quat):
        """Ruota un vettore nello spazio utilizzando un quaternione."""
        r = R.from_quat(quat)
        return r.apply(vector)

    def transform_velocity(velocity, rotation_matrix):
        """Transform velocity from IMU frame to global frame."""
        transformed_velocity = np.dot(rotation_matrix, velocity)
        return transformed_velocity

    def integrate_trapezoid(acc, previous_acc, dt, initial_velocity=np.zeros(3)):
        """Integrate acceleration using the trapezoidal rule to get velocity."""
        # Usa la media tra l'accelerazione precedente e quella attuale per l'integrazione
        velocity = initial_velocity + 0.5 * (previous_acc + acc) * dt
        return velocity

    def integrate(acc, dt, initial_velocity=np.zeros(3)):
        """Integrate acceleration to get velocity."""
        velocity = initial_velocity + acc * dt
        return velocity

    window_size = 300

    # Parse JSON data
    with open(file_path) as f:
        imu_data = json.load(f)

    # Initialize variables
    initial_velocity = np.zeros(3)
    previous_local_velocity = np.zeros(3)
    initial_position = np.zeros(3)
    initial_velocity_q = np.zeros(3)
    initial_position_q = np.zeros(3)
    trajectory_quaternion = []
    global_accelerations_quat = []
    velocities_quaternion = []
    quaternions = []
    previous_timestamp = None
    trajectory = []
    global_accelerations = []
    raw_accelerations_nog = []
    velocities = []  # Array per le velocità
    timestamps = []
    angular_velocities = []
    global_velocities = []

    # Raccolta dei dati grezzi (non filtrati)
    raw_accelerations = []
    raw_rotations = []
    raw_timestamps = []
    filtered_rotation_abs_relative = []


    for timestamp, data in sorted(imu_data.items()):
        timestamp = float(timestamp)
        orientation = [data['orientation']['x'],
                       data['orientation']['y'],
                       data['orientation']['z'],
                       data['orientation']['w']]  # Assumere ordine (x, y, z, w)

        linear_acceleration = np.array([data['linear_acceleration']['x'],
                                        data['linear_acceleration']['y'],
                                        data['linear_acceleration']['z']])

        angular_velocity = np.array([data['angular_velocity']['x'],
                                        data['angular_velocity']['y'],
                                        data['angular_velocity']['z']])

        euler_angles = quaternion_to_euler(orientation, euler_order)
        # zero_acc_axes = [False, False, False]  # Annulla X e Z
        # if zero_acc_axes:
        #     for i, zero in enumerate(zero_acc_axes):
        #         if zero:
        #             linear_acceleration[i] = 0.0  # Azzerare l'asse corrispondente
        #
        # # Convert quaternion to Euler angles with desired order
        #
        #
        # zero_euler_axes = [False, False, False]
        # if zero_euler_axes:
        #     for i, zero in enumerate(zero_euler_axes):
        #         if zero:
        #             euler_angles[i] = 0.0  # Azzerare l'angolo corrispondente

        # Conserva le accelerazioni, le rotazioni e i timestamp per il filtraggio
        raw_accelerations_nog.append(linear_acceleration)
        raw_rotations.append(euler_angles)
        raw_timestamps.append(timestamp)
        angular_velocities.append(angular_velocity)
        quaternions.append(orientation)

    # Converti le liste in array NumPy
    raw_accelerations = np.array(raw_accelerations_nog)
    raw_rotations = np.array(raw_rotations)
    raw_timestamps = np.array(raw_timestamps)
    angular_velocities = np.array(angular_velocities)
    quaternions = np.array(quaternions)





    # Filtrare le accelerazioni lineari con una finestra mobile
    filtered_accelerations = np.zeros((raw_accelerations.shape[0] - window_size + 1, raw_accelerations.shape[1]))
    for i in range(raw_accelerations.shape[1]):
        filtered_accelerations[:, i] = moving_average(raw_accelerations[:, i], window_size)

    # Filtrare le accelerazioni lineari con una finestra mobile
    filtered_rotations = np.zeros((raw_rotations.shape[0] - window_size + 1, raw_rotations.shape[1]))
    for i in range(raw_rotations.shape[1]):
        filtered_rotations[:, i] = moving_average(raw_rotations[:, i], window_size)





    # Rimuovere i valori iniziali da timestamps e rotazioni per allinearli alle accelerazioni filtrate
    filtered_timestamps = raw_timestamps[window_size-1:]  # Rimuove i primi (window_size-1) elementi
    filtered_angular_velocities = angular_velocities[window_size-1:]
    filtered_quaternions = quaternions[window_size - 1:]

    # Assuming filtered_accelerations has been calculated already, using its shape directly

    filtered_acceleration_v2 = np.zeros_like(filtered_accelerations)

    # Set x-axis values to 30 at indices 20, 21, and 22, y and z remain zero

    filtered_acceleration_v2[20:23, 0] = 200
    filtered_acceleration_v2[700:, 1] = 2

    #filtered_accelerations = filtered_acceleration_v2



    N = 10
    initial_gravity_vector = np.mean(filtered_accelerations[:N], axis=0)



    print("initial gravity:", initial_gravity_vector)

    previous_rot_mat = euler_to_rotation_matrix(filtered_rotations[0], euler_order)
    first_euler = filtered_rotations[0]
    # PRova a mettere angoli a zero prima


    dt_under_evaluation = []
    local_velocities = []


    # Iniziare l'integrazione usando solo i dati filtrati
    for timestamp, linear_acceleration, euler_angles, orientation in zip(filtered_timestamps, filtered_accelerations, filtered_rotations, filtered_quaternions):
        #euler_myrf = euler_angles-first_euler
        #euler_angles = - euler_angles
        euler_myrf = euler_angles
        if previous_timestamp is None:
            rotation_matrix = euler_to_rotation_matrix(euler_myrf, euler_order)
            initial_gravity_vector_global = transform_acceleration(initial_gravity_vector, rotation_matrix)
            global_acceleration = transform_acceleration(linear_acceleration, rotation_matrix)
            global_acceleration_ng = global_acceleration - initial_gravity_vector_global
            previous_acc_glob_ng = global_acceleration_ng
            previous_loc_acc_ng = linear_acceleration
            print("initial gravity global", initial_gravity_vector_global)
            print("first frame")




        if previous_timestamp is not None:

            rotation_matrix = euler_to_rotation_matrix(euler_myrf, euler_order)

            dt = timestamp - previous_timestamp
            #dt = 0.01
            dt_under_evaluation.append(dt)

            GLOBAL_APPROACH = 1
            if GLOBAL_APPROACH:

                global_acceleration = transform_acceleration(linear_acceleration, rotation_matrix)

                global_acceleration_ng = global_acceleration - initial_gravity_vector_global
                global_velocity = integrate_trapezoid(global_acceleration_ng,previous_acc_glob_ng, dt, initial_velocity)
                local_velocity = integrate_trapezoid(linear_acceleration,previous_loc_acc_ng, dt, previous_local_velocity)
            else:
                gravity_local = np.dot(rotation_matrix.T, initial_gravity_vector_global)
                net_acceleration_local = linear_acceleration - gravity_local

                global_acceleration_ng = transform_acceleration(net_acceleration_local, rotation_matrix)
                local_velocity = integrate(net_acceleration_local, dt, initial_velocity)
                global_velocity = transform_velocity(local_velocity,rotation_matrix)

            initial_position = np.array(initial_position)
            global_velocity = np.array(global_velocity)
            global_acceleration_ng = np.array(global_acceleration_ng)

            #position = initial_position + global_velocity * dt + 0.5 * linear_acceleration * dt ** 2
            position = initial_position + global_velocity * dt + 0.5 * global_acceleration_ng * (dt ** 2)
            trajectory.append(position)
            global_accelerations.append(global_acceleration_ng)


            # 2. Metodo dei quaternioni
            global_acc_quat = quaternion_rotate_vector(linear_acceleration, orientation)
            velocity_quaternion = integrate(global_acc_quat, dt, initial_velocity_q)
            position_quaternion = initial_position_q + velocity_quaternion * dt
            trajectory_quaternion.append(position_quaternion)
            velocities_quaternion.append(velocity_quaternion)
            global_accelerations_quat.append(global_acc_quat)
            local_velocities.append(local_velocity)
            global_velocities.append(global_velocity)
            filtered_rotation_abs_relative.append(euler_myrf)

            initial_velocity_q = velocity_quaternion
            initial_position_q = position_quaternion


            # Aggiornare lo stato per il prossimo ciclo
            initial_velocity = global_velocity
            initial_position = position
            previous_rot_mat = rotation_matrix
            previous_acc_glob_ng = global_acceleration_ng
            previous_loc_acc_ng = linear_acceleration
            previous_local_velocity = local_velocity




        previous_timestamp = timestamp

    # Rimuovere il primo elemento da tutte le liste per sincronizzarle
    filtered_rotations = filtered_rotations[1:]
    filtered_timestamps = filtered_timestamps[1:]
    filtered_accelerations = filtered_accelerations[1:]
    filtered_angular_velocities = filtered_angular_velocities[1:]
    filtered_quaternions =filtered_quaternions[1:]

    velocities = np.array(local_velocities)
    angles_deg = np.degrees(np.arctan2(velocities[:, 1], velocities[:, 0]))





    # Visualizzare i risultati
    # print(len(filtered_timestamps))
    # print(len(filtered_rotations))
    # print(len(global_accelerations))
    # print(len(velocities))  # Mostra la lunghezza del vettore delle velocità



    # plot_accelerations_and_rotations(filtered_timestamps, filtered_angular_velocities, filtered_rotations, "ANGULAR VELOCITY")
    #qplot_accelerations_and_rotations(filtered_timestamps, filtered_accelerations, filtered_rotations, "filtered")
    #plot_accelerations_and_rotations(filtered_timestamps, local_velocities, filtered_accelerations, filtered_rotation_abs_relative, "grandezze locali")
    #plot_accelerations_and_rotations(filtered_timestamps,global_velocities, global_accelerations, filtered_rotation_abs_relative, "grandezze globali")
    #plot_accelerations_and_rotations(filtered_timestamps, velocities, filtered_rotations)
    print("FREQUENZA IMU: ", calculate_frequencies(filtered_timestamps))
    # plot_translations(filtered_timestamps, trajectory, "IMU")
    # plot_translations(filtered_timestamps, trajectory_quaternion, "IMU_QUATERNIONS")


    return filtered_timestamps, trajectory, filtered_accelerations, global_accelerations, filtered_rotation_abs_relative
def load_json(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)


def plot_gnss_data(gnss_data):
    # Extracting x, y, z positions
    x = [entry['position']['x'] for entry in gnss_data.values()]
    y = [entry['position']['y'] for entry in gnss_data.values()]
    z = [entry['position']['z'] for entry in gnss_data.values()]




    # Creating a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the data
    ax.plot(x, y, z, marker='o')

    # Setting labels
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('Interpolated GNSS Data')

    # Setting equal scaling for all axes
    ax.set_box_aspect([np.ptp(x), np.ptp(y), np.ptp(z)])  # aspect ratio is 1:1:1 in data space

    # Show plot
    plt.show()


def interpolate_gnss_for_imu(gnss_data, imu_data, interpolation = False):
    # Extract timestamps and positions from GNSS data
    gnss_timestamps = [entry['timestamp'] for entry in gnss_data]
    gnss_positions_x = [entry['position']['x'] for entry in gnss_data]
    gnss_positions_y = [entry['position']['y'] for entry in gnss_data]
    gnss_positions_z = [entry['position']['z'] for entry in gnss_data]

    trajectory = np.stack((gnss_positions_x, gnss_positions_y, gnss_positions_z), axis=1)

    plot_translations(gnss_timestamps, trajectory, "GNSS RAW")
    print("FREQUENZA GNSS: ", calculate_frequencies(gnss_timestamps))

    # Create interpolation functions for x, y, z positions
    interp_x = interp1d(gnss_timestamps, gnss_positions_x, fill_value="extrapolate")
    interp_y = interp1d(gnss_timestamps, gnss_positions_y, fill_value="extrapolate")
    interp_z = interp1d(gnss_timestamps, gnss_positions_z, fill_value="extrapolate")





    # Create a new list to store the interpolated GNSS data for each IMU timestamp
    interpolated_gnss_data = {}


    if interpolation:

        for imu_timestamp in imu_data.keys():
            imu_timestamp_float = float(imu_timestamp)
            interpolated_position = {
                'x': float(interp_x(imu_timestamp_float)),
                'y': float(interp_y(imu_timestamp_float)),
                'z': float(interp_z(imu_timestamp_float))
            }
            interpolated_gnss_data[imu_timestamp] = {
                'position': interpolated_position
            }
        print("INTERPOLATION GNSS COMPLETED")
        return interpolated_gnss_data,gnss_timestamps, 0

    else:
        gnss_data_processed = {}
        for entry in gnss_data:
            timestamp = entry['timestamp']
            position = {
                'x': entry['position']['x'],
                'y': entry['position']['y'],
                'z': entry['position']['z']
            }
            gnss_data_processed[timestamp] = {
                'position': position
            }
        return gnss_data_processed, gnss_timestamps, trajectory

