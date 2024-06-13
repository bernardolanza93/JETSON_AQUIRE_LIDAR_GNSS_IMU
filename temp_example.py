import numpy as np
import matplotlib.pyplot as plt
import json
from scipy.spatial.transform import Rotation as R
from scipy.integrate import cumtrapz


def plot_trajectory_data(trajectory_data):
    timestamps = trajectory_data['timestamps']

    # Plot Euler Angles
    euler_angles = np.array(trajectory_data['euler_angles'])
    plt.figure()
    plt.plot(timestamps, euler_angles[:, 0], label='Roll')
    plt.plot(timestamps, euler_angles[:, 1], label='Pitch')
    plt.plot(timestamps, euler_angles[:, 2], label='Yaw')
    plt.title('Euler Angles over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot Displacements
    displacements = np.array(trajectory_data['displacements'])
    plt.figure()
    plt.plot(timestamps, displacements[:, 0], label='X')
    plt.plot(timestamps, displacements[:, 1], label='Y')
    plt.plot(timestamps, displacements[:, 2], label='Z')
    plt.title('Displacements over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Displacement (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot Raw Displacements
    raw_displacements = np.array(trajectory_data['raw_displacements'])
    plt.figure()
    plt.plot(timestamps, raw_displacements[:, 0], label='Raw X')
    plt.plot(timestamps, raw_displacements[:, 1], label='Raw Y')
    plt.plot(timestamps, raw_displacements[:, 2], label='Raw Z')
    plt.title('Raw Displacements over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Raw Displacement (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot Trajectory Points
    trajectory_points = np.array(trajectory_data['trajectory_points'])
    plt.figure()
    plt.plot(timestamps, trajectory_points[:, 0], label='X')
    plt.plot(timestamps, trajectory_points[:, 1], label='Y')
    plt.plot(timestamps, trajectory_points[:, 2], label='Z')
    plt.title('Trajectory Points over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)
    plt.show()


# ============================================================
# Rotazioni principali

# Rotazione di phi attorno al primo asse (=asse X) del frame inerziale
def r1(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), np.sin(phi)],
                     [0, -np.sin(phi), np.cos(phi)]])


# Seconda rotazione di theta attorno al secondo asse (=asse Y) del primo frame intermedio
def r2(theta):
    return np.array([[np.cos(theta), 0, -np.sin(theta)],
                     [0, 1, 0],
                     [np.sin(theta), 0, np.cos(theta)]])


# Terza rotazione di psi attorno al terzo asse (=asse Z) del secondo frame intermedio
def r3(psi):
    return np.array([[np.cos(psi), np.sin(psi), 0],
                     [-np.sin(psi), np.cos(psi), 0],
                     [0, 0, 1]])


# ====================================================
# Sequenza di rotazione e123

def q11(psi, theta):
    return np.cos(psi) * np.cos(theta)


def q12(psi, theta, phi):
    return np.cos(psi) * np.sin(theta) * np.sin(phi) + np.sin(psi) * np.cos(phi)


def q13(psi, theta, phi):
    return -np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi)


def q21(psi, theta):
    return - np.sin(psi) * np.cos(theta)


def q22(psi, theta, phi):
    return -np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi)


def q23(psi, theta, phi):
    return np.sin(psi) * np.sin(theta) * np.cos(phi) + np.cos(psi) * np.sin(phi)


def q31(theta):
    return np.sin(theta)


def q32(theta, phi):
    return - np.cos(theta) * np.sin(phi)


def q33(theta, phi):
    return np.cos(theta) * np.cos(phi)


def e123_dcm(psi, theta, phi):
    return np.array([[q11(psi, theta), q12(psi, theta, phi), q13(psi, theta, phi)],
                     [q21(psi, theta), q22(psi, theta, phi), q23(psi, theta, phi)],
                     [q31(theta), q32(theta, phi), q33(theta, phi)]])

# ====================================================
# Funzione per ruotare l'accelerazione utilizzando gli angoli di Eulero

def rotate_acceleration_manually(linear_acceleration, euler_angles):
    roll, pitch, yaw = euler_angles
    R = e123_dcm(yaw, pitch, roll)  # Utilizza la matrice di rotazione definita sopra
    rotated_acceleration = R @ linear_acceleration
    return rotated_acceleration


# Simula i dati IMU
def simulate_imu_data():
    imu_data = []
    imu_timestamps = np.linspace(0, 10, 100)  # 100 punti temporali in 10 secondi

    last_time = imu_timestamps[0]

    for t in imu_timestamps:
        dt = t - last_time

        last_time = t

        if t < 5:
            angular_velocity = np.array([0.0, 0.0, 0.0])  # Nessuna rotazione inizialmente
            linear_acceleration = np.array([0.0, 1.0, 0.0])  # Movimento costante lungo y
        else:
            # Simulate angular velocity with an inversion
            if t == imu_timestamps[len(imu_timestamps) // 2]:  # Simulate inversion at the midpoint
                angular_velocity = np.array([np.pi / dt, 0.0, 0.0])  # Change of pitch by pi radians
            else:
                angular_velocity = np.array([0.0, 0.0, 0.0])  # No rotation

            linear_acceleration = np.array([0.0, 1.0, 0.0])  # Movimento costante lungo y

        imu_data.append({
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        })

    return imu_data, imu_timestamps


# Funzione per ruotare manualmente l'accelerazione lineare in base agli angoli di Eulero
# Funzione per ruotare manualmente l'accelerazione lineare in base agli angoli di Eulero
# def rotate_acceleration_manually(linear_acceleration, euler_angles):
#     roll, pitch, yaw = euler_angles
#
#     # Matrice di rotazione attorno all'asse x (roll)
#     Rx = np.array([
#         [1, 0, 0],
#         [0, np.cos(roll), -np.sin(roll)],
#         [0, np.sin(roll), np.cos(roll)]
#     ])
#
#     # Matrice di rotazione attorno all'asse y (pitch)
#     Ry = np.array([
#         [np.cos(pitch), 0, np.sin(pitch)],
#         [0, 1, 0],
#         [-np.sin(pitch), 0, np.cos(pitch)]
#     ])
#
#     # Matrice di rotazione attorno all'asse z (yaw)
#     Rz = np.array([
#         [np.cos(yaw), -np.sin(yaw), 0],
#         [np.sin(yaw), np.cos(yaw), 0],
#         [0, 0, 1]
#     ])
#
#     # Combinare le matrici di rotazione
#     R = Rz @ Ry @ Rx
#
#     rotated_acceleration = R @ linear_acceleration
#
#     return rotated_acceleration

# Funzione per calcolare la traiettoria utilizzando i dati simulati
def compute_trajectory_zed(imu_data, imu_timestamps, output_file="trajectory_zed.json"):
    euler_angles = []
    displacements = []
    raw_displacements = []  # Raw displacements without transformation
    trajectory_points = []

    integrated_velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity
    integrated_displacement = np.array([0.0, 0.0, 0.0])  # Initial displacement

    last_time = imu_timestamps[0]

    # Inizializza l'orientamento iniziale
    orientation = R.from_euler('xyz', [0.0, 0.0, 0.0], degrees=False)
    orientations = [orientation]

    for i, imu_msg in enumerate(imu_data):
        current_time = imu_timestamps[i]
        dt = current_time - last_time
        last_time = current_time

        # Estrarre velocità angolare e accelerazione lineare
        angular_velocity = np.array(imu_msg['angular_velocity'])
        linear_acceleration = np.array(imu_msg['linear_acceleration'])

        # Calcolare il quaternione per l'incremento dell'orientamento
        angle = np.linalg.norm(angular_velocity) * dt
        if angle != 0:
            axis = angular_velocity / np.linalg.norm(angular_velocity)
            delta_q = R.from_rotvec(angle * axis)
            orientation = orientation * delta_q

        orientations.append(orientation)

        # Ruotare l'accelerazione utilizzando l'orientamento corrente
        rotated_acceleration = orientation.apply(linear_acceleration)

        # Integrare l'accelerazione ruotata per ottenere la velocità
        integrated_velocity += rotated_acceleration * dt

        # Integrare la velocità per ottenere lo spostamento
        integrated_displacement += integrated_velocity * dt

        # Salvare lo spostamento grezzo senza trasformazione
        raw_displacements.append(integrated_velocity * dt)

        # Salvare gli angoli di Eulero per il plot
        euler_angles.append(orientation.as_euler('xyz', degrees=False).tolist())
        displacements.append(integrated_displacement.copy())
        trajectory_points.append(integrated_displacement.copy())

        # Salvare la traiettoria e gli angoli di Eulero in un file
    trajectory_data = {
        'timestamps': imu_timestamps.tolist(),
        'euler_angles': euler_angles,
        'displacements': [disp.tolist() for disp in displacements],
        'raw_displacements': [raw_disp.tolist() for raw_disp in raw_displacements],
        'trajectory_points': [point.tolist() for point in trajectory_points]
    }



    with open(output_file, 'w') as f:
        json.dump(trajectory_data, f)

    return euler_angles, displacements, raw_displacements, trajectory_points



# Funzione per caricare e plottare la traiettoria
def plot_trajectory(filename="trajectory_zed.json"):
    with open(filename, 'r') as f:
        trajectory_data = json.load(f)

    euler_angles = np.array([np.array(angle) for angle in trajectory_data['euler_angles']])
    displacements = np.array([np.array(disp) for disp in trajectory_data['displacements']])
    trajectory_points = np.array([np.array(point) for point in trajectory_data['trajectory_points']])
    timestamps = np.array(trajectory_data['timestamps'])

    # Plot degli angoli di Eulero
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    axs[0].plot(timestamps, euler_angles[:, 0], label='Roll (radians)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Roll (radians)')
    axs[0].legend()

    axs[1].plot(timestamps, euler_angles[:, 1], label='Pitch (radians)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Pitch (radians)')
    axs[1].legend()

    axs[2].plot(timestamps, euler_angles[:, 2], label='Yaw (radians)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Yaw (radians)')
    axs[2].legend()

    plt.show()

    # Plot della traiettoria
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], label='Trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()


# Funzione principale per simulare i dati e calcolare la traiettoria
def main():
    imu_data, imu_timestamps = simulate_imu_data()
    euler_angles, displacements, raw_displacements, trajectory_points = compute_trajectory_zed(imu_data, imu_timestamps)
    plot_trajectory()


if __name__ == "__main__":
    main()
