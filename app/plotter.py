import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
def visualize_pc_with_trajectory(pc, trajectory_deltas):
    # Creare una finestra di visualizzazione per la pointcloud e la traiettoria
    point_size = 2
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud with Trajectory")

    # Aggiungi la pointcloud ricostruita
    vis.add_geometry(pc)

    # Imposta il colore di sfondo
    opt = vis.get_render_option()
    opt.background_color = np.array([0.1, 0.1, 0.1])  # Colore nero
    opt.point_size = point_size

    # Creare una lista di punti per rappresentare la traiettoria
    points = [[0, 0, 0]]  # Punto di partenza (origine)
    for timestamp in sorted(trajectory_deltas.keys()):
        delta = trajectory_deltas[timestamp]['slam_icp_transformation']
        # Aggiungi la nuova posizione come punto successivo
        last_position = points[-1]
        new_position = [
            last_position[0] + delta['delta_x'],
            last_position[1] + delta['delta_y'],
            last_position[2] + delta['delta_z']
        ]
        points.append(new_position)

    # Creare una LineSet per collegare i punti della traiettoria
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )

    # Aggiungi colori alla traiettoria per una migliore visualizzazione
    colors = [[1, 0, 0] for _ in lines]  # Rosso per tutte le linee
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # Aggiungi il LineSet alla visualizzazione
    vis.add_geometry(line_set)

    # Creare e aggiungere gli assi principali nel punto di origine
    axis_length = 0.8  # Lunghezza degli assi
    axis_points = [
        [0, 0, 0], [axis_length, 0, 0],  # Asse X in rosso
        [0, 0, 0], [0, axis_length, 0],  # Asse Y in verde
        [0, 0, 0], [0, 0, axis_length]   # Asse Z in blu
    ]
    axis_lines = [[0, 1], [2, 3], [4, 5]]
    axis_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Colori RGB per X, Y, Z

    # Creare un LineSet per gli assi
    axis_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(axis_points),
        lines=o3d.utility.Vector2iVector(axis_lines),
    )
    axis_line_set.colors = o3d.utility.Vector3dVector(axis_colors)

    # Aggiungi gli assi alla visualizzazione
    vis.add_geometry(axis_line_set)

    # Avvia la visualizzazione
    vis.run()
    vis.destroy_window()



def plot_GNSS_slamFPS_trajectories(downsampled_data):
    """
    Plot translational and rotational data from downsampled GNSS/IMU data over time.

    Parameters:
        downsampled_data (dict): Dictionary containing interpolated GNSS/IMU data.
    """
    # Ordinare i timestamp
    sorted_timestamps = sorted(downsampled_data.keys())
    timestamps = np.array(sorted_timestamps)

    # Estrarre le posizioni e le rotazioni dal dizionario
    positions = {'x': [], 'y': [], 'z': []}
    rotations = {'x': [], 'y': [], 'z': []}

    for t in sorted_timestamps:
        positions['x'].append(downsampled_data[t]['position']['x'])
        positions['y'].append(downsampled_data[t]['position']['y'])
        positions['z'].append(downsampled_data[t]['position']['z'])
        rotations['x'].append(downsampled_data[t]['rotation']['x'])
        rotations['y'].append(downsampled_data[t]['rotation']['y'])
        rotations['z'].append(downsampled_data[t]['rotation']['z'])

    # Creare subplots per le posizioni (x, y, z)
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    axs[0].plot(timestamps, positions['x'], label='Translazione X', color='b')
    axs[0].set_title('Posizione X nel Tempo')
    axs[0].set_xlabel('Timestamp')
    axs[0].set_ylabel('Posizione X [m]')
    axs[0].legend()

    axs[1].plot(timestamps, positions['y'], label='Translazione Y', color='g')
    axs[1].set_title('Posizione Y nel Tempo')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Posizione Y [m]')
    axs[1].legend()

    axs[2].plot(timestamps, positions['z'], label='Translazione Z', color='r')
    axs[2].set_title('Posizione Z nel Tempo')
    axs[2].set_xlabel('Timestamp')
    axs[2].set_ylabel('Posizione Z [m]')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

    # Creare subplots per le rotazioni (roll, pitch, yaw)
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    axs[0].plot(timestamps, rotations['x'], label='Rotazione X (Roll)', color='b')
    axs[0].set_title('Rotazione X (Roll) nel Tempo')
    axs[0].set_xlabel('Timestamp')
    axs[0].set_ylabel('Rotazione X [rad]')
    axs[0].legend()

    axs[1].plot(timestamps, rotations['y'], label='Rotazione Y (Pitch)', color='g')
    axs[1].set_title('Rotazione Y (Pitch) nel Tempo')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Rotazione Y [rad]')
    axs[1].legend()

    axs[2].plot(timestamps, rotations['z'], label='Rotazione Z (Yaw)', color='r')
    axs[2].set_title('Rotazione Z (Yaw) nel Tempo')
    axs[2].set_xlabel('Timestamp')
    axs[2].set_ylabel('Rotazione Z [rad]')
    axs[2].legend()

    plt.tight_layout()
    plt.show()


def plot_angles_2d_FROM_COUPLED_DELTAS(trajectory_deltas):
    import matplotlib.pyplot as plt

    # Ordinare i delta in base al timestamp
    sorted_timestamps = sorted(trajectory_deltas.keys())
    slam_angles = {'x': [], 'y': [], 'z': []}
    gnss_angles = {'x': [], 'y': [], 'z': []}

    # Angoli iniziali
    current_slam_angle = {'x': 0, 'y': 0, 'z': 0}
    current_gnss_angle = {'x': 0, 'y': 0, 'z': 0}

    # Calcolare gli angoli cumulativi a partire dai delta
    for timestamp in sorted_timestamps:
        slam_delta = trajectory_deltas[timestamp]['slam_icp_transformation']
        gnss_delta = trajectory_deltas[timestamp]['gnss_imu_transformation']

        # Aggiornare gli angoli SLAM
        current_slam_angle['x'] += slam_delta['delta_rotation_x']
        current_slam_angle['y'] += slam_delta['delta_rotation_y']
        current_slam_angle['z'] += slam_delta['delta_rotation_z']
        slam_angles['x'].append(current_slam_angle['x'])
        slam_angles['y'].append(current_slam_angle['y'])
        slam_angles['z'].append(current_slam_angle['z'])

        # Aggiornare gli angoli GNSS/IMU
        current_gnss_angle['x'] += gnss_delta['delta_rotation_x']
        current_gnss_angle['y'] += gnss_delta['delta_rotation_y']
        current_gnss_angle['z'] += gnss_delta['delta_rotation_z']
        gnss_angles['x'].append(current_gnss_angle['x'])
        gnss_angles['y'].append(current_gnss_angle['y'])
        gnss_angles['z'].append(current_gnss_angle['z'])

    # Creare subplots per gli angoli x, y, z
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    axs[0].plot(sorted_timestamps, slam_angles['x'], label='SLAM ICP Rotation X', color='b')
    axs[0].plot(sorted_timestamps, gnss_angles['x'], label='GNSS/IMU Rotation X', color='r')
    axs[0].set_title('Rotation X Over Time')
    axs[0].set_xlabel('Timestamp')
    axs[0].set_ylabel('Rotation X (radians)')
    axs[0].legend()

    axs[1].plot(sorted_timestamps, slam_angles['y'], label='SLAM ICP Rotation Y', color='b')
    axs[1].plot(sorted_timestamps, gnss_angles['y'], label='GNSS/IMU Rotation Y', color='r')
    axs[1].set_title('Rotation Y Over Time')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Rotation Y (radians)')
    axs[1].legend()

    axs[2].plot(sorted_timestamps, slam_angles['z'], label='SLAM ICP Rotation Z', color='b')
    axs[2].plot(sorted_timestamps, gnss_angles['z'], label='GNSS/IMU Rotation Z', color='r')
    axs[2].set_title('Rotation Z Over Time')
    axs[2].set_xlabel('Timestamp')
    axs[2].set_ylabel('Rotation Z (radians)')
    axs[2].legend()

    plt.tight_layout()
    plt.show()


def plot_3d_trajectory_from_coupled_deltas(trajectory_deltas):
    """
    Plot the 3D trajectory reconstructed from coupled deltas for both SLAM and GNSS/IMU.

    Parameters:
        trajectory_deltas (dict): Dictionary containing SLAM and GNSS/IMU deltas with timestamps as keys.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Ordinare i delta in base al timestamp
    sorted_timestamps = sorted(trajectory_deltas.keys())
    slam_positions = {'x': [], 'y': [], 'z': []}
    gnss_positions = {'x': [], 'y': [], 'z': []}

    # Posizioni iniziali
    current_slam_position = {'x': 0, 'y': 0, 'z': 0}
    current_gnss_position = {'x': 0, 'y': 0, 'z': 0}

    # Calcolare le posizioni cumulative a partire dai delta
    for timestamp in sorted_timestamps:
        slam_delta = trajectory_deltas[timestamp]['slam_icp_transformation']
        gnss_delta = trajectory_deltas[timestamp]['gnss_imu_transformation']

        # Aggiornare la posizione SLAM
        current_slam_position['x'] += slam_delta['delta_x']
        current_slam_position['y'] += slam_delta['delta_y']
        current_slam_position['z'] += slam_delta['delta_z']
        slam_positions['x'].append(current_slam_position['x'])
        slam_positions['y'].append(current_slam_position['y'])
        slam_positions['z'].append(current_slam_position['z'])

        # Aggiornare la posizione GNSS/IMU
        current_gnss_position['x'] += gnss_delta['delta_x']
        current_gnss_position['y'] += gnss_delta['delta_y']
        current_gnss_position['z'] += gnss_delta['delta_z']
        gnss_positions['x'].append(current_gnss_position['x'])
        gnss_positions['y'].append(current_gnss_position['y'])
        gnss_positions['z'].append(current_gnss_position['z'])

        # Creare il plot 3D
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot SLAM trajectory
    ax.plot(slam_positions['x'], slam_positions['y'], slam_positions['z'], label='SLAM Trajectory', color='b')

    # Plot GNSS-IMU trajectory
    ax.plot(gnss_positions['x'], gnss_positions['y'], gnss_positions['z'], label='GNSS/IMU Trajectory', color='r',
            linestyle='--')

    # Set labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory from Coupled Deltas (SLAM vs GNSS/IMU)')
    ax.legend()

    # Ensure the axes have the same scale
    max_range = max(
        max(slam_positions['x'] + gnss_positions['x']) - min(slam_positions['x'] + gnss_positions['x']),
        max(slam_positions['y'] + gnss_positions['y']) - min(slam_positions['y'] + gnss_positions['y']),
        max(slam_positions['z'] + gnss_positions['z']) - min(slam_positions['z'] + gnss_positions['z'])
    ) / 2.0

    mid_x = (max(slam_positions['x'] + gnss_positions['x']) + min(slam_positions['x'] + gnss_positions['x'])) * 0.5
    mid_y = (max(slam_positions['y'] + gnss_positions['y']) + min(slam_positions['y'] + gnss_positions['y'])) * 0.5
    mid_z = (max(slam_positions['z'] + gnss_positions['z']) + min(slam_positions['z'] + gnss_positions['z'])) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Show plot
    plt.show()

def plot_trajectory_2d_FROM_COUPLED_DELTAS(trajectory_deltas):
    import matplotlib.pyplot as plt

    # Ordinare i delta in base al timestamp
    sorted_timestamps = sorted(trajectory_deltas.keys())
    slam_positions = {'x': [], 'y': [], 'z': []}
    gnss_positions = {'x': [], 'y': [], 'z': []}

    # Posizioni iniziali
    current_slam_position = {'x': 0, 'y': 0, 'z': 0}
    current_gnss_position = {'x': 0, 'y': 0, 'z': 0}

    # Calcolare le posizioni cumulative a partire dai delta
    for timestamp in sorted_timestamps:
        slam_delta = trajectory_deltas[timestamp]['slam_icp_transformation']
        gnss_delta = trajectory_deltas[timestamp]['gnss_imu_transformation']

        # Aggiornare la posizione SLAM
        current_slam_position['x'] += slam_delta['delta_x']
        current_slam_position['y'] += slam_delta['delta_y']
        current_slam_position['z'] += slam_delta['delta_z']
        slam_positions['x'].append(current_slam_position['x'])
        slam_positions['y'].append(current_slam_position['y'])
        slam_positions['z'].append(current_slam_position['z'])

        # Aggiornare la posizione GNSS/IMU
        current_gnss_position['x'] += gnss_delta['delta_x']
        current_gnss_position['y'] += gnss_delta['delta_y']
        current_gnss_position['z'] += gnss_delta['delta_z']
        gnss_positions['x'].append(current_gnss_position['x'])
        gnss_positions['y'].append(current_gnss_position['y'])
        gnss_positions['z'].append(current_gnss_position['z'])

    # Creare subplots per x, y, z
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    axs[0].plot(sorted_timestamps, slam_positions['x'], label='SLAM ICP X', color='b')
    axs[0].plot(sorted_timestamps, gnss_positions['x'], label='GNSS/IMU X', color='r')
    axs[0].set_title('X Position Over Time')
    axs[0].set_xlabel('Timestamp')
    axs[0].set_ylabel('X Position')
    axs[0].legend()

    axs[1].plot(sorted_timestamps, slam_positions['y'], label='SLAM ICP Y', color='b')
    axs[1].plot(sorted_timestamps, gnss_positions['y'], label='GNSS/IMU Y', color='r')
    axs[1].set_title('Y Position Over Time')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Y Position')
    axs[1].legend()

    axs[2].plot(sorted_timestamps, slam_positions['z'], label='SLAM ICP Z', color='b')
    axs[2].plot(sorted_timestamps, gnss_positions['z'], label='GNSS/IMU Z', color='r')
    axs[2].set_title('Z Position Over Time')
    axs[2].set_xlabel('Timestamp')
    axs[2].set_ylabel('Z Position')
    axs[2].legend()

    plt.tight_layout()
    plt.show()


def plot_3d_trajectory_GNSSSIMU_SLAM(downsampled_data, slam_data):
    """
    Plot the 3D trajectory comparison between GNSS-IMU interpolated positions and SLAM positions.

    Parameters:
        downsampled_data (dict): Dictionary containing downsampled GNSS-IMU positions with timestamps as keys.
        slam_data (dict): Dictionary containing SLAM data with timestamps as keys.
    """
    # Extract SLAM timestamps, x, y, and z positions
    slam_timestamps = sorted(slam_data.keys())
    current_position = np.eye(4)
    x_slam = [current_position[0, 3]]
    y_slam = [current_position[1, 3]]
    z_slam = [current_position[2, 3]]

    # Applica le trasformazioni in ordine di timestamp
    for ts in slam_timestamps:
        transform = slam_data[ts]
        current_position = np.dot(current_position, transform)
        x_slam.append(current_position[0, 3])
        y_slam.append(current_position[1, 3])
        z_slam.append(current_position[2, 3])

    # Extract downsampled GNSS-IMU positions
    x_gnss_imu = [downsampled_data[t]['position']['x'] for t in slam_timestamps if t in downsampled_data]
    y_gnss_imu = [downsampled_data[t]['position']['y'] for t in slam_timestamps if t in downsampled_data]
    z_gnss_imu = [downsampled_data[t]['position']['z'] for t in slam_timestamps if t in downsampled_data]

    # Create 3D plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot SLAM trajectory
    ax.plot(x_slam[:len(x_gnss_imu)], y_slam[:len(y_gnss_imu)], z_slam[:len(z_gnss_imu)], label='SLAM Trajectory', color='b')

    # Plot GNSS-IMU trajectory
    ax.plot(x_gnss_imu, y_gnss_imu, z_gnss_imu, label='GNSS-IMU Trajectory', color='r', linestyle='--')

    # Set labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory Comparison (GNSS-IMU vs SLAM)')
    ax.legend()

    # Show plot
    plt.show()


def plot_gnss_imu_vs_slam(downsampled_data, slam_data):
    """
    Plot the comparison between GNSS-IMU interpolated positions and SLAM positions.

    Parameters:
        downsampled_data (dict): Dictionary containing downsampled GNSS-IMU positions with timestamps as keys.
        slam_data (dict): Dictionary containing SLAM data with timestamps as keys.
    """
    # Extract SLAM timestamps, x, y, and z positions
    slam_timestamps = sorted(slam_data.keys())
    current_position = np.eye(4)
    x_slam = [current_position[0, 3]]
    y_slam = [current_position[1, 3]]
    z_slam = [current_position[2, 3]]

    # Applica le trasformazioni in ordine di timestamp
    for ts in slam_timestamps:
        transform = slam_data[ts]
        current_position = np.dot(current_position, transform)
        x_slam.append(current_position[0, 3])
        y_slam.append(current_position[1, 3])
        z_slam.append(current_position[2, 3])

    # Extract downsampled GNSS-IMU positions
    x_gnss_imu = [downsampled_data[t]['position']['x'] for t in slam_timestamps if t in downsampled_data]
    y_gnss_imu = [downsampled_data[t]['position']['y'] for t in slam_timestamps if t in downsampled_data]
    z_gnss_imu = [downsampled_data[t]['position']['z'] for t in slam_timestamps if t in downsampled_data]

    # Create subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Plot X positions
    axs[0].plot(slam_timestamps[:len(x_gnss_imu)], x_slam[:len(x_gnss_imu)], label='SLAM X Position', color='b')
    axs[0].plot(slam_timestamps[:len(x_gnss_imu)], x_gnss_imu, label='GNSS-IMU X Position', color='r', linestyle='--')
    axs[0].set_xlabel('Timestamp')
    axs[0].set_ylabel('X Position (m)')
    axs[0].set_title('Comparison of X Positions (GNSS-IMU vs SLAM)')
    axs[0].legend()

    # Plot Y positions
    axs[1].plot(slam_timestamps[:len(y_gnss_imu)], y_slam[:len(y_gnss_imu)], label='SLAM Y Position', color='b')
    axs[1].plot(slam_timestamps[:len(y_gnss_imu)], y_gnss_imu, label='GNSS-IMU Y Position', color='r', linestyle='--')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Y Position (m)')
    axs[1].set_title('Comparison of Y Positions (GNSS-IMU vs SLAM)')
    axs[1].legend()

    # Plot Z positions
    axs[2].plot(slam_timestamps[:len(z_gnss_imu)], z_slam[:len(z_gnss_imu)], label='SLAM Z Position', color='b')
    axs[2].plot(slam_timestamps[:len(z_gnss_imu)], z_gnss_imu, label='GNSS-IMU Z Position', color='r', linestyle='--')
    axs[2].set_xlabel('Timestamp')
    axs[2].set_ylabel('Z Position (m)')
    axs[2].set_title('Comparison of Z Positions (GNSS-IMU vs SLAM)')
    axs[2].legend()

    # Improve layout
    plt.tight_layout()
    plt.show()




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
def plot_3d_trajectory_with_arrows(interpolated_data):
    """
    Plot 3D trajectory of positions and associate arrows every 200 elements indicating orientation based on rotations.
    Highlight the starting point of the trajectory.

    Parameters:
        interpolated_data (dict): Dictionary containing timestamps, rotations, and positions.
    """
    # Extract data from the dictionary
    timestamps = list(interpolated_data.keys())
    positions = np.array([[interpolated_data[t]['position']['x'],
                           interpolated_data[t]['position']['y'],
                           interpolated_data[t]['position']['z']] for t in timestamps])
    rotations = np.array([[interpolated_data[t]['rotation']['x'],
                           interpolated_data[t]['rotation']['y'],
                           interpolated_data[t]['rotation']['z']] for t in timestamps])

    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the entire trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory', color='b')

    # Highlight the starting point
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], color='g', s=100, label='Start Point', marker='o')

    # Plot arrows every 200 points based on rotations
    arrow_length = 2.6  # Length of the arrow for visualization, increased by 30%
    for i in range(0, len(positions), 60):
        position = positions[i]
        rotation = R.from_euler('xyz', rotations[i])  # Rotations are in radians

        # Directions for x, y, z axes in the local frame
        x_direction = rotation.apply([1, 0, 0])
        y_direction = rotation.apply([0, 1, 0])
        z_direction = rotation.apply([0, 0, 1])

        # Plot arrows for x, y, z axes
        ax.quiver(position[0], position[1], position[2],
                  x_direction[0], x_direction[1], x_direction[2],
                  length=arrow_length, color='r', label='X Axis' if i == 0 else "")
        ax.quiver(position[0], position[1], position[2],
                  y_direction[0], y_direction[1], y_direction[2],
                  length=arrow_length, color='g', label='Y Axis' if i == 0 else "")
        ax.quiver(position[0], position[1], position[2],
                  z_direction[0], z_direction[1], z_direction[2],
                  length=arrow_length, color='b', label='Z Axis' if i == 0 else "")

    # Set labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory with Orientation Arrows and Start Point Highlighted')

    # Setting equal scaling for all axes
    ax.set_box_aspect([np.ptp(positions[:, 0]), np.ptp(positions[:, 1]), np.ptp(positions[:, 2])])

    ax.legend()
    plt.show()

def plot_interpolated_data(interpolated_data):
    """
    Plot positions (x, y, z) and rotations (x, y, z) in function of timestamps.

    Parameters:
        interpolated_data (dict): Dictionary containing timestamps, rotations, and positions.
    """
    # Extract data from the dictionary
    timestamps = list(interpolated_data.keys())
    positions_x = [interpolated_data[t]['position']['x'] for t in timestamps]
    positions_y = [interpolated_data[t]['position']['y'] for t in timestamps]
    positions_z = [interpolated_data[t]['position']['z'] for t in timestamps]
    rotations_x = [interpolated_data[t]['rotation']['x'] for t in timestamps]
    rotations_y = [interpolated_data[t]['rotation']['y'] for t in timestamps]
    rotations_z = [interpolated_data[t]['rotation']['z'] for t in timestamps]

    # Create subplots for positions and rotations
    fig, ax = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Positions and Rotations Over Time', fontsize=16)

    # Plot positions
    ax[0, 0].plot(timestamps, positions_x, label='X Position')
    ax[0, 0].set_xlabel('Timestamp (s)')
    ax[0, 0].set_ylabel('Position (m)')
    ax[0, 0].set_title('X Position')

    ax[0, 1].plot(timestamps, positions_y, label='Y Position')
    ax[0, 1].set_xlabel('Timestamp (s)')
    ax[0, 1].set_ylabel('Position (m)')
    ax[0, 1].set_title('Y Position')

    ax[0, 2].plot(timestamps, positions_z, label='Z Position')
    ax[0, 2].set_xlabel('Timestamp (s)')
    ax[0, 2].set_ylabel('Position (m)')
    ax[0, 2].set_title('Z Position')

    # Plot rotations
    ax[1, 0].plot(timestamps, rotations_x, label='X Rotation')
    ax[1, 0].set_xlabel('Timestamp (s)')
    ax[1, 0].set_ylabel('Rotation (rad)')
    ax[1, 0].set_title('X Rotation')

    ax[1, 1].plot(timestamps, rotations_y, label='Y Rotation')
    ax[1, 1].set_xlabel('Timestamp (s)')
    ax[1, 1].set_ylabel('Rotation (rad)')
    ax[1, 1].set_title('Y Rotation')

    ax[1, 2].plot(timestamps, rotations_z, label='Z Rotation')
    ax[1, 2].set_xlabel('Timestamp (s)')
    ax[1, 2].set_ylabel('Rotation (rad)')
    ax[1, 2].set_title('Z Rotation')

    # Improve layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

def plot_angles_over_time(interpolated_data):
    """
    Plot roll, pitch, and yaw angles in function of timestamps.

    Parameters:
        interpolated_data (dict): Dictionary containing timestamps, rotations, and positions.
    """
    # Extract data from the dictionary
    timestamps = list(interpolated_data.keys())
    roll = [interpolated_data[t]['rotation']['x'] for t in timestamps]
    pitch = [interpolated_data[t]['rotation']['y'] for t in timestamps]
    yaw = [interpolated_data[t]['rotation']['z'] for t in timestamps]

    # Create subplots for roll, pitch, and yaw angles
    fig, ax = plt.subplots(3, 1, figsize=(10, 15))
    fig.suptitle('Roll, Pitch, and Yaw Angles Over Time', fontsize=16)

    # Plot roll angle
    ax[0].plot(timestamps, roll, label='Roll Angle')
    ax[0].set_xlabel('Timestamp (s)')
    ax[0].set_ylabel('Angle (rad)')
    ax[0].set_title('Roll Angle')
    ax[0].legend()

    # Plot pitch angle
    ax[1].plot(timestamps, pitch, label='Pitch Angle')
    ax[1].set_xlabel('Timestamp (s)')
    ax[1].set_ylabel('Angle (rad)')
    ax[1].set_title('Pitch Angle')
    ax[1].legend()

    # Plot yaw angle
    ax[2].plot(timestamps, yaw, label='Yaw Angle')
    ax[2].set_xlabel('Timestamp (s)')
    ax[2].set_ylabel('Angle (rad)')
    ax[2].set_title('Yaw Angle')
    ax[2].legend()

    # Improve layout
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()
