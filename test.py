import json
import sys

import matplotlib.pyplot as plt
import numpy as np


def plot_trajectory(trajectory_points):
    trajectory_points = np.array(trajectory_points)
    x = trajectory_points[:, 0]
    y = trajectory_points[:, 1]
    z = trajectory_points[:, 2]

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    axs[0].plot(x, y, label='x vs y')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].legend()

    axs[1].plot(x, z, label='x vs z')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Z')
    axs[1].legend()

    axs[2].plot(y, z, label='y vs z')
    axs[2].set_xlabel('Y')
    axs[2].set_ylabel('Z')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

def load_imu_data_from_file(filename="imu_data.json"):
    with open(filename, 'r') as f:
        data = json.load(f)
    return data


def integrate_angular_velocity(angular_velocity, timestamps):
    angles = [0]  # Start with an initial angle of 0
    for i in range(1, len(angular_velocity)):
        dt = timestamps[i] - timestamps[i - 1]
        angles.append(angles[-1] + angular_velocity[i] * dt)
    return angles


def integrate_linear_acceleration(acceleration, timestamps):
    velocities = [0]  # Start with an initial velocity of 0
    displacements = [0]  # Start with an initial displacement of 0
    for i in range(1, len(acceleration)):
        dt = timestamps[i] - timestamps[i - 1]
        velocities.append(velocities[-1] + acceleration[i] * dt)
        displacements.append(displacements[-1] + velocities[-1] * dt)
    return displacements

def load_trajectory_points(filename="trajectory_points.json"):
    with open(filename, 'r') as f:
        trajectory_points = json.load(f)
    # Convert lists back to numpy arrays
    trajectory_points = [np.array(point) for point in trajectory_points]
    return trajectory_points


def plot_imu_data_from_file(filename="imu_data.json"):
    data = load_imu_data_from_file(filename)

    imu_timestamps = data["timestamps"]
    linear_acceleration_x = data["linear_acceleration_x"]
    linear_acceleration_y = data["linear_acceleration_y"]
    linear_acceleration_z = data["linear_acceleration_z"]
    angular_velocity_x = data["angular_velocity_x"]
    angular_velocity_y = data["angular_velocity_y"]
    angular_velocity_z = data["angular_velocity_z"]

    roll = integrate_angular_velocity(angular_velocity_x, imu_timestamps)
    pitch = integrate_angular_velocity(angular_velocity_y, imu_timestamps)
    yaw = integrate_angular_velocity(angular_velocity_z, imu_timestamps)

    displacement_x = integrate_linear_acceleration(linear_acceleration_x, imu_timestamps)
    displacement_y = integrate_linear_acceleration(linear_acceleration_y, imu_timestamps)
    displacement_z = integrate_linear_acceleration(linear_acceleration_z, imu_timestamps)

    fig, axs = plt.subplots(3, 2, figsize=(15, 10))

    axs[0, 0].plot(imu_timestamps, displacement_x, label='Displacement X')
    axs[0, 0].set_xlabel('Timestamp')
    axs[0, 0].set_ylabel('Displacement X (m)')
    axs[0, 0].legend()

    axs[0, 1].plot(imu_timestamps, roll, label='Roll')
    axs[0, 1].set_xlabel('Timestamp')
    axs[0, 1].set_ylabel('Roll (degrees)')
    axs[0, 1].legend()

    axs[1, 0].plot(imu_timestamps, displacement_y, label='Displacement Y')
    axs[1, 0].set_xlabel('Timestamp')
    axs[1, 0].set_ylabel('Displacement Y (m)')
    axs[1, 0].legend()

    axs[1, 1].plot(imu_timestamps, pitch, label='Pitch')
    axs[1, 1].set_xlabel('Timestamp')
    axs[1, 1].set_ylabel('Pitch (degrees)')
    axs[1, 1].legend()

    axs[2, 0].plot(imu_timestamps, displacement_z, label='Displacement Z')
    axs[2, 0].set_xlabel('Timestamp')
    axs[2, 0].set_ylabel('Displacement Z (m)')
    axs[2, 0].legend()

    axs[2, 1].plot(imu_timestamps, yaw, label='Yaw')
    axs[2, 1].set_xlabel('Timestamp')
    axs[2, 1].set_ylabel('Yaw (degrees)')
    axs[2, 1].legend()

    plt.tight_layout()
    plt.show()


def load_raw_displacements_zed(filename="trajectory_zed.json"):
    with open(filename, 'r') as f:
        trajectory_data = json.load(f)
    raw_displacements = [np.array(disp) for disp in trajectory_data['raw_displacements']]
    return raw_displacements

def plot_raw_displacements_zed(raw_displacements):
    raw_displacements = np.array(raw_displacements)
    x = np.cumsum(raw_displacements[:, 0])
    y = np.cumsum(raw_displacements[:, 1])
    z = np.cumsum(raw_displacements[:, 2])

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    axs[0].plot(x, y, label='x vs y')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].axis('equal')
    axs[0].legend()

    axs[1].plot(x, z, label='x vs z')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Z')
    axs[1].axis('equal')
    axs[1].legend()

    axs[2].plot(y, z, label='y vs z')
    axs[2].set_xlabel('Y')
    axs[2].set_ylabel('Z')
    axs[2].axis('equal')
    axs[2].legend()

    plt.tight_layout()
    plt.show()


def load_imu_data(filename="imu_xsens_data.json"):
    with open(filename, 'r') as f:
        imu_data = json.load(f)
    return imu_data


def calculate_displacement(imu_data):
    positions = []
    velocities = np.array([0.0, 0.0, 0.0])
    position = np.array([0.0, 0.0, 0.0])
    last_time = imu_data[0]['timestamp']

    for entry in imu_data:
        print(entry)
        current_time = entry['timestamp']
        dt = current_time - last_time
        last_time = current_time

        linear_acceleration = np.array(entry['linear_acceleration'])
        velocities += linear_acceleration * dt
        position += velocities * dt
        positions.append(position.copy())

    return np.array(positions)


def plot_trajectory(positions):
    positions = np.array(positions)  # Convert list of numpy arrays to a single numpy array
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))

    axs[0].plot(x, y, label='x vs y')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    axs[0].legend()

    axs[1].plot(x, z, label='x vs z')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Z')
    axs[1].legend()

    axs[2].plot(y, z, label='y vs z')
    axs[2].set_xlabel('Y')
    axs[2].set_ylabel('Z')
    axs[2].legend()

    plt.tight_layout()
    plt.show()


def load_gnss_data(filename="gnss_data.json"):
    with open(filename, 'r') as f:
        gnss_data = json.load(f)
    return gnss_data


def parse_gnss_data(gnss_data):
    lats = []
    lons = []
    for entry in gnss_data:
        data = entry['data']
        parts = data.split(',')
        if 'GNGGA' in parts[0]:
            lat = float(parts[2])
            lat_dir = parts[3]
            lon = float(parts[4])
            lon_dir = parts[5]

            # Convert latitude and longitude to decimal degrees
            lat_deg = int(lat / 100)
            lat_min = lat - lat_deg * 100
            lat = lat_deg + lat_min / 60

            lon_deg = int(lon / 100)
            lon_min = lon - lon_deg * 100
            lon = lon_deg + lon_min / 60

            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon

            lats.append(lat)
            lons.append(lon)

    return lats, lons

def plot_gnss_trajectory(lats, lons):
    plt.figure(figsize=(10, 6))
    plt.plot(lons, lats, marker='o', linestyle='-', color='b')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GNSS Trajectory')
    plt.grid(True)
    plt.show()

import json

def load_trajectory_zed(filename="trajectory_zed.json"):
    with open(filename, 'r') as f:
        trajectory_data = json.load(f)
    trajectory_points = [np.array(point) for point in trajectory_data['acceleration']]
    euler_angles = [np.array(angle) for angle in trajectory_data['euler_angles']]
    return trajectory_points, euler_angles



def plot_trajectory_and_angles_zed(filename="trajectory_zed.json"):
    with open(filename, 'r') as f:
        trajectory_data = json.load(f)

    euler_angles = np.array([np.array(angle) for angle in trajectory_data['euler_angles']])
    displacements = np.array([np.array(disp) for disp in trajectory_data['displacements']])
    rotated_accelerations = np.array([np.array(rot_acc) for rot_acc in trajectory_data['rotated_accelerations']])
    trajectory_points = np.array([np.array(point) for point in trajectory_data['trajectory_points']])
    timestamps = np.array(trajectory_data['timestamps'])

    # Plot degli angoli di Eulero, traiettorie, e accelerazioni ruotate
    fig, axs = plt.subplots(4, 3, figsize=(18, 24))

    axs[0, 0].plot(timestamps, euler_angles[:, 0], label='Roll (radians)')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Roll (radians)')
    axs[0, 0].legend()

    axs[0, 1].plot(timestamps, euler_angles[:, 1], label='Pitch (radians)')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Pitch (radians)')
    axs[0, 1].legend()

    axs[0, 2].plot(timestamps, euler_angles[:, 2], label='Yaw (radians)')
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel('Yaw (radians)')
    axs[0, 2].legend()

    # Plot delle traiettorie
    axs[1, 0].plot(displacements[:, 0], displacements[:, 1], label='Trajectory (X vs Y)')
    axs[1, 0].set_xlabel('X')
    axs[1, 0].set_ylabel('Y')
    axs[1, 0].axis('equal')
    axs[1, 0].legend()

    axs[1, 1].plot(displacements[:, 0], displacements[:, 2], label='Trajectory (X vs Z)')
    axs[1, 1].set_xlabel('X')
    axs[1, 1].set_ylabel('Z')
    axs[1, 1].axis('equal')
    axs[1, 1].legend()

    axs[1, 2].plot(displacements[:, 1], displacements[:, 2], label='Trajectory (Y vs Z)')
    axs[1, 2].set_xlabel('Y')
    axs[1, 2].set_ylabel('Z')
    axs[1, 2].axis('equal')
    axs[1, 2].legend()

    # Plot delle accelerazioni ruotate
    axs[2, 0].plot(timestamps, rotated_accelerations[:, 0], label='Rotated Acc X')
    axs[2, 0].set_xlabel('Time (s)')
    axs[2, 0].set_ylabel('Rotated Acc X')
    axs[2, 0].legend()

    axs[2, 1].plot(timestamps, rotated_accelerations[:, 1], label='Rotated Acc Y')
    axs[2, 1].set_xlabel('Time (s)')
    axs[2, 1].set_ylabel('Rotated Acc Y')
    axs[2, 1].legend()

    axs[2, 2].plot(timestamps, rotated_accelerations[:, 2], label='Rotated Acc Z')
    axs[2, 2].set_xlabel('Time (s)')
    axs[2, 2].set_ylabel('Rotated Acc Z')
    axs[2, 2].legend()

    # Plot dei displacements in funzione del tempo
    axs[3, 0].plot(timestamps, displacements[:, 0], label='Displacement X')
    axs[3, 0].set_xlabel('Time (s)')
    axs[3, 0].set_ylabel('Displacement X')
    axs[3, 0].legend()

    axs[3, 1].plot(timestamps, displacements[:, 1], label='Displacement Y')
    axs[3, 1].set_xlabel('Time (s)')
    axs[3, 1].set_ylabel('Displacement Y')
    axs[3, 1].legend()

    axs[3, 2].plot(timestamps, displacements[:, 2], label='Displacement Z')
    axs[3, 2].set_xlabel('Time (s)')
    axs[3, 2].set_ylabel('Displacement Z')
    axs[3, 2].legend()

    plt.tight_layout()
    plt.show()


# Step 3: Load and plot the trajectory

plot_trajectory_and_angles_zed()


raw_displacements =  load_raw_displacements_zed()

plot_raw_displacements_zed(raw_displacements)

sys.exit()

gnss_data = load_gnss_data("gnss_data.json")
lats, lons = parse_gnss_data(gnss_data)
plot_gnss_trajectory(lats, lons)
loaded_trajectory_points = load_trajectory_points("trajectory_points.json")



# Plot IMU data from the saved file
plot_imu_data_from_file()

# Load trajectory points from file

# Plot trajectory points
plot_trajectory(loaded_trajectory_points)
sys.exit()

imu_data = load_imu_data("imu_xsens_data.json")
positions = calculate_displacement(imu_data)
plot_trajectory(positions)