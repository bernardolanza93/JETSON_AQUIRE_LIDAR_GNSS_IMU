import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Parametri di simulazione
total_time = 20  # Tempo totale della simulazione in secondi
dt = 0.01  # Intervallo di tempo in secondi
steps = int(total_time / dt)

# Fase 1: Accelerazione fino a velocità costante
acc_local = np.zeros((steps, 3))
acc_local[:int(steps / 5), 0] = 0.1  # Accelerazione iniziale in x fino al 20% del tempo

# Angoli di rotazione di Eulero
angles = np.zeros((steps, 3))  # Nessuna rotazione inizialmente

# Definiamo i parametri per l'inversione
n_secondi_inizio_rotazione = 5  # Inizio della rotazione dopo 5 secondi
m_secondi_durata_rotazione = 3  # Durata della rotazione 3 secondi
inizio_rotazione = int(n_secondi_inizio_rotazione / dt)
fine_rotazione = inizio_rotazione + int(m_secondi_durata_rotazione / dt)

# Rotazione graduale di 180 gradi attorno all'asse Y
for t in range(inizio_rotazione, fine_rotazione):
    angles[t, 1] = (t - inizio_rotazione) / (fine_rotazione - inizio_rotazione) * 180  # Rotazione graduale fino a 180°

# Mantieni a 180° dopo la rotazione
angles[fine_rotazione:, 1] = 180


# Funzione per trasformare la velocità locale in globale
def transform_velocity(velocity_local, angles, order='xyz'):
    rotation = R.from_euler(order, angles, degrees=True)
    velocity_global = rotation.apply(velocity_local)
    return velocity_global


# Integrazione per ottenere la velocità e la posizione globali
def integrate(acc_local, dt):
    velocity = np.zeros_like(acc_local)
    position = np.zeros_like(acc_local)

    # Integrare per ottenere velocità e posizione
    for t in range(1, len(acc_local)):
        velocity[t] = velocity[t - 1] + acc_local[t] * dt
        position[t] = position[t - 1] + velocity[t] * dt
    return velocity, position


# Fase 1: Integrare l'accelerazione per ottenere la velocità e la posizione locali
velocity_local, position = integrate(acc_local, dt)

# Fase 2: Trasformare la velocità locale in globale, applicando le rotazioni graduali
velocity_global = np.array([transform_velocity(velocity_local[t], angles[t]) for t in range(steps)])

# Fase 3: Integrare la velocità globale per ottenere la posizione finale
for t in range(1, len(velocity_global)):
    position[t] = position[t - 1] + velocity_global[t] * dt

# Grafico della traiettoria finale (posizione)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], label="Traiettoria")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Traiettoria con Inversione di Movimento")
plt.show()