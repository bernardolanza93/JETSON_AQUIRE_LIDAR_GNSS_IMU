imu_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/localization_data/imu_data.json'
gnss_file = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/localization_data/gnss_data.json'
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import json


def correct_orientation(accelerations, quaternions):
    rotations = R.from_quat(quaternions)
    corrected_accelerations = rotations.apply(accelerations)

    # Rimuovi la componente gravitazionale
    gravity = np.array([0, 0, 9.81])  # Direzione della gravità nel sistema globale
    corrected_accelerations -= rotations.apply(gravity)

    return corrected_accelerations

# Funzione per caricare i dati JSON
def load_json(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

# Funzione per calcolare una traiettoria IMU tra due punti GNSS
def reconstruct_imu_trajectory_between_gnss(p1, p2, imu_data, imu_timestamps):
    start_time = p1['timestamp']
    end_time = p2['timestamp']

    # Filtra i dati IMU tra i due timestamp GNSS
    indices = np.where((imu_timestamps >= start_time) & (imu_timestamps <= end_time))[0]
    if len(indices) == 0:
        print(f"Nessun dato IMU tra {start_time} e {end_time}")
        return np.array([])

    timestamps = imu_timestamps[indices]
    accelerations = imu_data['linear_acceleration'][indices]
    orientations = imu_data['orientation'][indices]

    # Correggi le accelerazioni con le orientazioni
    if len(accelerations) == 0 or len(orientations) == 0:
        print(f"Dati mancanti per accelerazioni o orientazioni tra {start_time} e {end_time}")
        return np.array([])

    corrected_accelerations = correct_orientation(accelerations, orientations)

    # Integrazione per ottenere velocità e posizioni relative
    velocities = np.zeros_like(corrected_accelerations)
    positions = np.zeros_like(corrected_accelerations)

    for i in range(1, len(timestamps)):
        delta_t = timestamps[i] - timestamps[i - 1]
        velocities[i] = velocities[i - 1] + corrected_accelerations[i - 1] * delta_t
        positions[i] = positions[i - 1] + velocities[i - 1] * delta_t + 0.5 * corrected_accelerations[i - 1] * delta_t**2

    # Scala e allinea la traiettoria IMU al vettore GNSS
    gnss_vector = np.array([p2['position']['x'], p2['position']['y'], p2['position']['z']]) - \
                  np.array([p1['position']['x'], p1['position']['y'], p1['position']['z']])
    imu_vector = positions[-1] - positions[0]

    if np.linalg.norm(imu_vector) == 0:
        print(f"Il vettore IMU è nullo tra {start_time} e {end_time}")
        return np.array([])

    scale_factor = np.linalg.norm(gnss_vector) / np.linalg.norm(imu_vector)
    aligned_positions = positions * scale_factor

    # Trasla per iniziare esattamente da P1
    aligned_positions += np.array([p1['position']['x'], p1['position']['y'], p1['position']['z']]) - aligned_positions[0]

    return aligned_positions



gnss_data = load_json(gnss_file)
imu_data_raw = load_json(imu_file)


# Prepara i dati IMU
imu_timestamps = np.array(list(map(float, imu_data_raw.keys())))
imu_data = {
    'orientation': np.array([[data['orientation']['x'], data['orientation']['y'], data['orientation']['z'], data['orientation']['w']] for data in imu_data_raw.values()]),
    'linear_acceleration': np.array([[data['linear_acceleration']['x'], data['linear_acceleration']['y'], data['linear_acceleration']['z']] for data in imu_data_raw.values()])
}

# Ricostruisci la traiettoria GNSS + IMU con specifica degli indici
# start_index = 0  # Specifica l'indice iniziale
# end_index = len(gnss_data) - 1  # Specifica l'indice finale
start_index = 600  # Specifica l'indice iniziale
end_index = 1000  # Specifica l'indice finale

final_trajectory = []
for i in range(start_index, end_index):
    p1 = gnss_data[i]
    p2 = gnss_data[i + 1]

    segment_trajectory = reconstruct_imu_trajectory_between_gnss(p1, p2, imu_data, imu_timestamps)
    if final_trajectory:
        segment_trajectory = segment_trajectory[1:]  # Evita duplicati nei punti di collegamento
    final_trajectory.extend(segment_trajectory)

final_trajectory = np.array(final_trajectory)

# Visualizza la traiettoria
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(final_trajectory[:, 0], final_trajectory[:, 1], final_trajectory[:, 2], label='Traiettoria GNSS + IMU')
ax.set_xlabel('Posizione X (m)')
ax.set_ylabel('Posizione Y (m)')
ax.set_zlabel('Posizione Z (m)')
ax.set_title('Traiettoria combinata GNSS e IMU')
ax.legend()
plt.show()
