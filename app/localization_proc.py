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


def plot_trajectory_3d_imuu(timestamps, trajectory):
    trajectory = np.array(trajectory)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], marker='o')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('3D Trajectory')

    # Setting equal scaling for all axes
    ax.set_box_aspect([np.ptp(trajectory[:, 0]), np.ptp(trajectory[:, 1]), np.ptp(trajectory[:, 2])])

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

def plot_accelerations_and_rotations(timestamps, accelerations, rotations,label = "no data", window_size=1):
    accelerations = np.array(accelerations)
    rotations = np.array(rotations)

    # Filtrare accelerazioni con una finestra mobile
    filtered_accelerations = np.zeros((accelerations.shape[0] - window_size + 1, accelerations.shape[1]))
    for i in range(accelerations.shape[1]):
        filtered_accelerations[:, i] = moving_average(accelerations[:, i], window_size)

    # Filtrare rotazioni con una finestra mobile
    filtered_rotations = np.zeros((rotations.shape[0] - window_size + 1, rotations.shape[1]))
    for i in range(rotations.shape[1]):
        filtered_rotations[:, i] = moving_average(rotations[:, i], window_size)

    filtered_timestamps = timestamps[:filtered_accelerations.shape[0]]

    time_labels = timestamp_to_hms(filtered_timestamps)

    # Creare figure con 6 subplots
    fig, ax = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f'Spostamenti X,Y,Z sensore: {label}', fontsize=16)  # Titolo generale


    # Plottare accelerazioni
    ax[0].plot(time_labels, filtered_accelerations[:, 0], label='X Acceleration (filtered)')
    ax[0].set_ylabel('X Acceleration (m/s^2)')
    #ax[0].set_xlim([start, ending])
    ax[0].legend()

    ax[1].plot(time_labels, filtered_accelerations[:, 1], label='Y Acceleration (filtered)')
    ax[1].set_ylabel('Y Acceleration (m/s^2)')
    #ax[1].set_xlim([start, ending])
    ax[1].legend()

    ax[2].plot(time_labels, filtered_accelerations[:, 2], label='Z Acceleration (filtered)')
    ax[2].set_ylabel('Z Acceleration (m/s^2)')
    #ax[2].set_xlim([start, ending])
    ax[2].legend()


    # Plottare rotazioni
    ax[3].plot(time_labels, filtered_rotations[:, 0], label='X Rotation (filtered)')
    ax[3].set_ylabel('X Rotation (rad/s)')
    ax[3].legend()
    #ax[3].set_xlim([start, ending])

    ax[4].plot(time_labels, filtered_rotations[:, 1], label='Y Rotation (filtered)')
    ax[4].set_ylabel('Y Rotation (rad/s)')
    #ax[4].set_xlim([start, ending])
    ax[4].legend()

    ax[5].plot(time_labels, filtered_rotations[:, 2], label='Z Rotation (filtered)')
    ax[5].set_ylabel('Z Rotation (rad/s)')
    ax[5].set_xlabel('Timestamp (s)')
    #ax[5].set_xlim([start,ending])
    ax[5].legend()

    # Usa MaxNLocator per limitare il numero di tick visibili sull'asse X
    for axis in ax:
        axis.xaxis.set_major_locator(MaxNLocator(nbins=10))  # Mostra al massimo 10 etichette sull'asse X

    # Ruota le etichette per renderle leggibili
    plt.setp(ax[2].get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")


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


def process_imu_data_v2(file_path, euler_order='yxz', degrees=False):
    def euler_to_rotation_matrix(angles, order):
        """Convert Euler angles to a rotation matrix with specified order."""
        return R.from_euler(order, angles).as_matrix()

    def quaternion_to_euler(q, order, degrees=False):
        """Convert a quaternion into Euler angles with the specified order."""
        r = R.from_quat(q)
        return r.as_euler(order, degrees=degrees)

    def transform_acceleration(acc, rotation_matrix):
        """Transform acceleration from IMU frame to global frame."""
        return np.dot(rotation_matrix, acc)

    def integrate(acc, dt, initial_velocity=np.zeros(3)):
        """Integrate acceleration to get velocity."""
        velocity = initial_velocity + acc * dt
        return velocity

    window_size = 50

    # Parse JSON data
    with open(file_path) as f:
        imu_data = json.load(f)

    # Initialize variables
    initial_velocity = np.zeros(3)
    initial_position = np.zeros(3)
    previous_timestamp = None
    trajectory = []
    global_accelerations = []
    velocities = []  # Array per le velocità
    timestamps = []

    # Raccolta dei dati grezzi (non filtrati)
    raw_accelerations = []
    raw_rotations = []
    raw_timestamps = []

    for timestamp, data in sorted(imu_data.items()):
        timestamp = float(timestamp)
        orientation = [data['orientation']['x'],
                       data['orientation']['y'],
                       data['orientation']['z'],
                       data['orientation']['w']]  # Assumere ordine (x, y, z, w)

        linear_acceleration = np.array([data['linear_acceleration']['x'],
                                        data['linear_acceleration']['y'],
                                        data['linear_acceleration']['z']])

        # Convert quaternion to Euler angles with desired order
        euler_angles = quaternion_to_euler(orientation, euler_order, degrees=True)

        # Conserva le accelerazioni, le rotazioni e i timestamp per il filtraggio
        raw_accelerations.append(linear_acceleration)
        raw_rotations.append(euler_angles)
        raw_timestamps.append(timestamp)

    # Convertire in numpy array
    raw_accelerations = np.array(raw_accelerations)
    raw_rotations = np.array(raw_rotations)
    raw_timestamps = np.array(raw_timestamps)

    # Fase 1: Calcolare la media delle prime N letture (per esempio, 10) per ottenere il vettore di gravità
    N = 10  # Prendi i primi 10 campioni
    gravity_vector = np.mean(raw_accelerations[:N], axis=0)

    # Sottrai il vettore di gravità da tutte le letture di accelerazione
    raw_accelerations = raw_accelerations - gravity_vector

    # Filtrare le accelerazioni lineari con una finestra mobile
    filtered_accelerations = np.zeros((raw_accelerations.shape[0] - window_size + 1, raw_accelerations.shape[1]))
    for i in range(raw_accelerations.shape[1]):
        filtered_accelerations[:, i] = moving_average(raw_accelerations[:, i], window_size)

    # Rimuovere i valori iniziali da timestamps e rotazioni per allinearli alle accelerazioni filtrate
    filtered_timestamps = raw_timestamps[window_size-1:]  # Rimuove i primi (window_size-1) elementi
    filtered_rotations = raw_rotations[window_size-1:]  # Rimuove i primi (window_size-1) elementi

    # Iniziare l'integrazione usando solo i dati filtrati
    for timestamp, linear_acceleration, euler_angles in zip(filtered_timestamps, filtered_accelerations, filtered_rotations):

        # Convertire gli angoli di Eulero filtrati in matrice di rotazione
        rotation_matrix = euler_to_rotation_matrix(euler_angles, euler_order)

        if previous_timestamp is not None:
            dt = timestamp - previous_timestamp

            # Trasformare l'accelerazione nel sistema globale
            global_acceleration = transform_acceleration(linear_acceleration, rotation_matrix)

            # Integra l'accelerazione globale per ottenere la velocità globale
            velocity = integrate(global_acceleration, dt, initial_velocity)

            # Calcola la nuova posizione utilizzando l'accelerazione globale e la velocità globale
            position = initial_position + velocity * dt + 0.5 * global_acceleration * dt ** 2

            trajectory.append(position)
            velocities.append(velocity)  # Salvare la velocità calcolata
            global_accelerations.append(global_acceleration)

            # Aggiornare lo stato per il prossimo ciclo
            initial_velocity = velocity
            initial_position = position

        previous_timestamp = timestamp

    # Rimuovere il primo elemento da tutte le liste per sincronizzarle
    filtered_rotations = filtered_rotations[1:]
    filtered_timestamps = filtered_timestamps[1:]
    filtered_accelerations = filtered_accelerations[1:]

    # Visualizzare i risultati
    print(len(filtered_timestamps))
    print(len(filtered_rotations))
    print(len(global_accelerations))
    print(len(velocities))  # Mostra la lunghezza del vettore delle velocità
    plot_accelerations_and_rotations(filtered_timestamps, filtered_accelerations, filtered_rotations)
    plot_accelerations_and_rotations(filtered_timestamps, global_accelerations, filtered_rotations)
    plot_accelerations_and_rotations(filtered_timestamps, velocities, filtered_rotations)
    plot_translations(filtered_timestamps, trajectory)

    return filtered_timestamps, trajectory, filtered_accelerations, global_accelerations, filtered_rotations


def process_imu_data(file_path, euler_order='xyz', degrees=False):
    #yxz
    def euler_to_rotation_matrix(angles, order):
        """Convert Euler angles to a rotation matrix with specified order."""
        return R.from_euler(order, angles).as_matrix()
    def transform_acceleration(acc, rotation_matrix):
        """Transform acceleration from IMU frame to global frame."""
        return np.dot(rotation_matrix, acc)

    def quaternion_to_euler(q, order, degrees=False):
        """Convert a quaternion into Euler angles with the specified order."""
        r = R.from_quat(q)
        return r.as_euler(order, degrees=degrees)


    def quaternion_rotate_vector(vector, quat):
        """Ruota un vettore nello spazio utilizzando un quaternione."""
        r = R.from_quat(quat)
        return r.apply(vector)

    def transform_velocity(velocity, rotation_matrix):
        """Transform velocity from IMU frame to global frame."""
        transformed_velocity = np.dot(rotation_matrix, velocity)
        return transformed_velocity

    def integrate(acc, dt, initial_velocity=np.zeros(3)):
        """Integrate acceleration to get velocity."""
        velocity = initial_velocity + acc * dt
        return velocity

    def rotate_gravity(gravity_vector, delta_rotation_matrix):
        """
        Ruota il vettore di gravità usando la matrice di rotazione relativa.

        gravity_vector: Vettore di gravità da ruotare.
        delta_rotation_matrix: Matrice di rotazione relativa da applicare.
        """
        # Ruota la gravità usando la matrice di rotazione relativa
        return np.dot(delta_rotation_matrix.T, gravity_vector)

    window_size = 50

    # Parse JSON data
    with open(file_path) as f:
        imu_data = json.load(f)

    # Initialize variables
    initial_velocity = np.zeros(3)
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

    # Raccolta dei dati grezzi (non filtrati)
    raw_accelerations = []
    raw_rotations = []
    raw_timestamps = []


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

        euler_angles = quaternion_to_euler(orientation, euler_order, degrees=False)
        # zero_acc_axes = [False, True, True]  # Annulla X e Z
        # if zero_acc_axes:
        #     for i, zero in enumerate(zero_acc_axes):
        #         if zero:
        #             linear_acceleration[i] = 0.0  # Azzerare l'asse corrispondente
        #
        # #Convert quaternion to Euler angles with desired order
        #
        #
        # zero_euler_axes = [True, True, False]
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
    initial_orientation = quaternions[0]  # Quaternione iniziale


    # Convertire in numpy array
    raw_accelerations = np.array(raw_accelerations_nog)
    raw_rotations = np.array(raw_rotations)
    raw_timestamps = np.array(raw_timestamps)
    angular_velocities = np.array(angular_velocities)
    quaternions = np.array(quaternions)

    # # Fase 1: Calcolare la media delle prime N letture (per esempio, 10) per ottenere il vettore di gravità
    # N = 10  # Prendi i primi 10 campioni
    # gravity_vector = np.mean(raw_accelerations_nog[:N], axis=0)
    # print(gravity_vector)
    #
    # # Sottrai il vettore di gravità da tutte le letture di accelerazi0one
    # raw_accelerations = raw_accelerations_nog - gravity_vector


    # Filtrare le accelerazioni lineari con una finestra mobile
    filtered_accelerations = np.zeros((raw_accelerations.shape[0] - window_size + 1, raw_accelerations.shape[1]))
    for i in range(raw_accelerations.shape[1]):
        filtered_accelerations[:, i] = moving_average(raw_accelerations[:, i], window_size)

    # Rimuovere i valori iniziali da timestamps e rotazioni per allinearli alle accelerazioni filtrate
    filtered_timestamps = raw_timestamps[window_size-1:]  # Rimuove i primi (window_size-1) elementi
    filtered_rotations = raw_rotations[window_size-1:]  # Rimuove i primi (window_size-1) elementi
    filtered_angular_velocities = angular_velocities[window_size-1:]
    filtered_quaternions = quaternions[window_size - 1:]

    # Convertire gli angoli di Eulero filtrati in matrice di rotazione

    N = 10
    initial_gravity_vector_n10_local = np.mean(filtered_accelerations[:N], axis=0)
    print("initial gravity local:", initial_gravity_vector_n10_local)

    previous_rot_mat = euler_to_rotation_matrix(filtered_rotations[0], euler_order)

    dt_under_evaluation = []


    # Iniziare l'integrazione usando solo i dati filtrati
    for timestamp, linear_acceleration, euler_angles, orientation in zip(filtered_timestamps, filtered_accelerations, filtered_rotations, filtered_quaternions):
        if previous_timestamp is  None:

            first_euler = euler_angles
            rotation_matrix = euler_to_rotation_matrix(euler_angles, euler_order)
            # rotation= R.from_quat(orientation)
            # rotation_matrix = rotation.as_matrix()
            first_frame_accelaeration_gravity = initial_gravity_vector_n10_local
            initial_gravity_vector_global = transform_acceleration(first_frame_accelaeration_gravity, rotation_matrix)
            print("initial gravity global:", initial_gravity_vector_global)


            print("first frame")




        if previous_timestamp is not None:
            rotation_matrix = euler_to_rotation_matrix(euler_angles, euler_order)
            rotation_matrix_relative_to_first_frame = euler_to_rotation_matrix(euler_angles-first_euler, euler_order)
            gravity_from_first_frame_to_local = transform_acceleration(first_frame_accelaeration_gravity,rotation_matrix_relative_to_first_frame)

            # rotation= R.from_quat(orientation)
            # rotation_matrix = rotation.as_matrix()



            dt = timestamp - previous_timestamp
            #dt = 0.01
            dt_under_evaluation.append(dt)

            #trasformo acc gravita globaloe in locale:
            #local_gravity = np.dot(rotation_matrix.T, initial_gravity_vector)
            #linear_acceleration_ng = linear_acceleration - local_gravity
            GLOBAL_APP = 0
            if GLOBAL_APP:
                global_acceleration = transform_acceleration(linear_acceleration, rotation_matrix)
                global_acceleration_ng = global_acceleration - initial_gravity_vector_global
                global_velocity = integrate(global_acceleration_ng, dt, initial_velocity)
                position = initial_position + global_velocity * dt + 0.5 * global_acceleration_ng * dt ** 2
            else:
                delta_rotation_matrix = np.dot(rotation_matrix, np.linalg.inv(previous_rot_mat))

                #gravity_vector = rotate_gravity(initial_gravity_vector_n10_local, delta_rotation_matrix)
                # print(gravity_vector)
                linear_acceleration_nog = linear_acceleration - initial_gravity_vector_n10_local
                velocity = integrate(linear_acceleration_nog, dt, initial_velocity)
                global_velocity = transform_velocity(velocity, rotation_matrix)
                global_acceleration_ng = transform_acceleration(linear_acceleration_nog, rotation_matrix)
                position = initial_position + global_velocity * dt + 0.5 * global_acceleration_ng * dt ** 2


            trajectory.append(position)
            velocities.append(global_velocity)  # Salvare la velocità calcolata
            global_accelerations.append(global_acceleration_ng)


            # 2. Metodo dei quaternioni
            global_acc_quat = quaternion_rotate_vector(linear_acceleration, orientation)
            velocity_quaternion = integrate(global_acc_quat, dt, initial_velocity_q)
            position_quaternion = initial_position_q + velocity_quaternion * dt
            trajectory_quaternion.append(position_quaternion)
            velocities_quaternion.append(velocity_quaternion)
            global_accelerations_quat.append(global_acc_quat)

            initial_velocity_q = velocity_quaternion
            initial_position_q = position_quaternion


            # Aggiornare lo stato per il prossimo ciclo
            initial_velocity = global_velocity
            initial_position = position
            previous_rot_mat = rotation_matrix




        previous_timestamp = timestamp

    # Rimuovere il primo elemento da tutte le liste per sincronizzarle
    filtered_rotations = filtered_rotations[1:]
    filtered_timestamps = filtered_timestamps[1:]
    filtered_accelerations = filtered_accelerations[1:]
    filtered_angular_velocities = filtered_angular_velocities[1:]
    filtered_quaternions =filtered_quaternions[1:]


    # Visualizzare i risultati
    # print(len(filtered_timestamps))
    # print(len(filtered_rotations))
    # print(len(global_accelerations))
    # print(len(velocities))  # Mostra la lunghezza del vettore delle velocità



    plot_accelerations_and_rotations(filtered_timestamps, velocities, filtered_rotations, "VELOCITY")
    # plot_accelerations_and_rotations(filtered_timestamps, filtered_angular_velocities, filtered_rotations, "ANGULAR VELOCITY")
    #qplot_accelerations_and_rotations(filtered_timestamps, filtered_accelerations, filtered_rotations, "filtered")
    plot_accelerations_and_rotations(filtered_timestamps, filtered_accelerations, filtered_rotations, "RAW ACC")
    plot_accelerations_and_rotations(filtered_timestamps, global_accelerations, filtered_rotations, "GLOBAL ACC")
    #plot_accelerations_and_rotations(filtered_timestamps, velocities, filtered_rotations)
    print("FREQUENZA IMU: ", calculate_frequencies(filtered_timestamps))
    # plot_translations(filtered_timestamps, trajectory, "IMU")
    # plot_translations(filtered_timestamps, trajectory_quaternion, "IMU_QUATERNIONS")


    return filtered_timestamps, trajectory, filtered_accelerations, global_accelerations, filtered_rotations

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

