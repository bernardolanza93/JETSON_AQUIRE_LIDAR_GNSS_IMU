import re
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.optimize import least_squares
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA

folder_pc = "/home/mmt-ben/Documents/spheres"
# Mappa dei raggi reali
real_radius_map = {
    "1": 0.12,
    "2": 0.06,
    "3": 0.0485,
    "4": 0.0385,
    "5": 0.028,
    "6": 0.024
}
Percentile_limit = 95







def normalize_point_cloud(pcd):
    points = np.asarray(pcd.points)
    center = np.mean(points, axis=0)
    normalized_points = points - center
    pcd.points = o3d.utility.Vector3dVector(normalized_points)
    return pcd



def sphere_residuals(params, points):
    x0, y0, z0, r = params
    residuals = np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2) - r
    return residuals

def calculate_sphericity(points, radius_95):
    hull = ConvexHull(points)
    volume_hull = hull.volume
    area_hull = hull.area

    volume_sphere = (4 / 3) * np.pi * (radius_95 ** 3)
    area_sphere = 4 * np.pi * (radius_95 ** 2)

    sphericity = (np.pi ** (1/3) * (6 * volume_hull) ** (2/3)) / area_hull

    return sphericity


def advanced_uncertanty(points, percentile_limit=Percentile_limit):
    # Calcola il centroide della nuvola di punti
    centroid = np.mean(points, axis=0)

    # Calcola le distanze radiali dei punti dal centroide
    distances = np.linalg.norm(points - centroid, axis=1)

    # Superficie includente del 95%
    radius_95 = np.percentile(distances, percentile_limit)

    # Filtra i punti che rientrano nel 95° percentile
    filtered_points = points[distances <= radius_95]

    # Calcola il convesso usando i punti filtrati
    hull = ConvexHull(filtered_points)
    volume_hull = hull.volume

    # Calcola il volume della sfera corrispondente al raggio del 95° percentile
    volume_sphere = (4 / 3) * np.pi * (radius_95 ** 3)

    # Calcola il volume mancante
    volume_missing = volume_sphere - volume_hull

    # Incertezza volumetrica (approssimativamente uguale al volume mancante)
    delta_V = volume_missing

    # Propagazione dell'incertezza per ottenere l'incertezza del raggio
    delta_r = delta_V / (4 * np.pi * radius_95**2)

    # Stima del raggio con incertezza
    r_stim = radius_95
    r_stim_upper = r_stim + delta_r / 2
    r_stim_lower = r_stim - delta_r / 2
    uncert_98 = (delta_r/2) * 2

    sphericity = calculate_sphericity(points,radius_95)

    #print(f"Incertezza del Raggio: {delta_r:.4f}")
    return uncert_98, r_stim, sphericity, centroid[0], centroid[1], centroid[2]





def fit_sphere(points, percentile=Percentile_limit):

    """
    Fit a sphere to a set of points and calculate the radius as the percentile of all distances from the center.

    Args:
    - points (np.ndarray): The input points as a numpy array of shape (N, 3).
    - percentile (int): The percentile to use for the radius calculation (default is 90).

    Returns:
    - center (tuple): The center of the fitted sphere.
    - radius (float): The radius of the fitted sphere.
    """
    # Calcola il centro dei punti
    center = np.mean(points, axis=0)

    # Calcola le distanze dei punti dal centro
    distances = np.linalg.norm(points - center, axis=1)

    # Calcola il raggio come il percentile specificato delle distanze
    radius = np.percentile(distances, percentile)

    return center[0],center[1],center[2], radius




def estimate_uncertainty(points, center, radius):
    distances = np.linalg.norm(points - center, axis=1)
    residuals = distances - radius
    uncertainty = np.std(residuals)
    return uncertainty


def remove_outliers(points, eps=0.05, min_samples=10):
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    # Consider points with label -1 as outliers
    mask = labels != -1
    print("removed DBSCAN:", len(points)-len(points[mask]), "/",len(points))
    return points[mask]


def estimate_external_uncertainty(points, center, radius):
    distances = np.linalg.norm(points - center, axis=1)
    external_points = distances > radius
    external_residuals = distances[external_points] - radius
    external_uncertainty = np.std(external_residuals)
    return external_uncertainty, external_residuals

# Funzione per calcolare i residui della sfera
def sphere_residuals(params, points):
    x0, y0, z0, r = params
    residuals = np.sqrt((points[:, 0] - x0)**2 + (points[:, 1] - y0)**2 + (points[:, 2] - z0)**2) - r
    return residuals


def calculate_density_distribution(points, x0, y0, z0, r):
    distances = np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2)
    p95 = np.percentile(distances, Percentile_limit)
    mask_95 = distances < p95
    points_95 = points[mask_95]

    inertia_tensor = np.zeros((3, 3))
    center = np.array([x0, y0, z0])
    for point in points_95:
        r = point - center
        inertia_tensor += np.outer(r, r)

    eigenvalues, _ = np.linalg.eigh(inertia_tensor)
    density_distribution = eigenvalues / np.sum(eigenvalues)

    return density_distribution

# Funzione per calcolare la sfericità usando il Convex Hull
# Funzione per calcolare la sfericità usando il Convex Hull
def calculate_convex_hull_sphericity(points, x0, y0, z0, r):
    # Filtra i punti che si trovano all'interno della sfera
    distances = np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2)
    mask = distances <= r
    points_within_sphere = points[mask]

    if len(points_within_sphere) < 4:
        return 0  # Non abbastanza punti per calcolare il Convex Hull

    hull = ConvexHull(points_within_sphere)
    hull_volume = hull.volume

    sphere_volume = (4 / 3) * np.pi * r ** 3
    sphericity = hull_volume / sphere_volume



    return sphericity


def calculate_density(points, x0, y0, z0, r):
    # Volume della sfera
    sphere_volume = (4 / 3) * np.pi * (r) ** 3

    # Filtrare i punti all'interno della sfera
    distances = np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2 + (points[:, 2] - z0) ** 2)
    points_within_sphere = points[distances <= r]

    # Numero di punti all'interno della sfera
    num_points_within_sphere = len(points_within_sphere)

    # Densità media
    density = num_points_within_sphere / sphere_volume

    return density
def extract_sphere_id(filename):
    if len(filename) > 2:
        return filename[2]
    else:
        raise ValueError(f"Cannot extract sphere ID from filename: {filename}")

def plot_point_cloud_and_sphere(points, center, radius,filepath):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot point cloud
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='orange', s=1)

    # Create a sphere
    u, v = np.mgrid[0:2 * np.pi:100j, 0:np.pi:50j]
    x = center[0] + radius * np.cos(u) * np.sin(v)
    y = center[1] + radius * np.sin(u) * np.sin(v)
    z = center[2] + radius * np.cos(v)

    # Plot the sphere
    ax.plot_surface(x, y, z, color='b', alpha=0.3, rstride=1, cstride=1)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Point Cloud ' + str(filepath.split('/')[-1]) + ' points:' + str(len(points)))

    # Set aspect ratio to be equal
    ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio

    # Equal scaling for all axes
    max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                          points[:, 1].max() - points[:, 1].min(),
                          points[:, 2].max() - points[:, 2].min()]).max() / 2.0

    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()
# Iterate over all point clouds in the directory
import os


# Liste per memorizzare i dati
real_radii = []
estimated_radii = []
uncertainties = []
file_types = []
point_counts = []
sphericity = []
densities = []

density_distributions = []
PLOT_SPHERES_FIT = 0

directory = folder_pc
# Ottieni la lista dei file .ply nella directory e ordina in ordine alfabetico
ply_files = sorted([f for f in os.listdir(directory) if f.endswith(".ply")])

# Itera su ogni file ordinato
for filename in ply_files:
    file_path = os.path.join(directory, filename)
    try:
        sphere_id = extract_sphere_id(filename)

        pcd = o3d.io.read_point_cloud(file_path)
        pcd = normalize_point_cloud(pcd)
        real_radius = real_radius_map[sphere_id]



        # Fit the sphere
        points = np.asarray(pcd.points)


        # Fit the sphere

        uncertanty , r , hull_sphericity ,x0, y0, z0  = advanced_uncertanty(points)

        density_distribution = calculate_density_distribution(points, x0, y0, z0, r)
        # hull_sphericity = calculate_convex_hull_sphericity(points, x0, y0, z0, r)
        density = calculate_density(points,x0, y0, z0, r)
        print("_______________________",filename)
        print(f"Estimated Radius: {r:.4f} Real: {real_radius}")
        print(f"Hull Sphericity: {hull_sphericity}")
        print(f"uncertanty: {uncertanty}")

        # Calculate the radius difference
        radius_difference = r - real_radius

        # Print results


        # Determina il tipo di file
        if 'sv' in file_path:
            file_types.append('sv')
        elif 'sf' in file_path:
            file_types.append('sf')
        elif 'sa' in file_path:
            file_types.append('sa')

        real_radii.append(real_radius)
        estimated_radii.append(r)
        uncertainties.append(uncertanty)
        point_counts.append(len(points))
        sphericity.append(hull_sphericity)
        densities.append(density)


        # Plot the point cloud and the fitted sphere
        if PLOT_SPHERES_FIT:

            plot_point_cloud_and_sphere(points, [x0, y0, z0], r,file_path )
    except ValueError as e:
        print(e)


# Mappa dei colori con nomi personalizzati
color_map = {'sv': ('red', 'Viametris'), 'sf': ('blue', 'KA double side'), 'sa': ('green', 'KA single view')}
colors = [color_map[ft][0] for ft in file_types]

# Definiamo piccoli spostamenti per evitare sovrapposizioni
shifts = {'sv': -0.0005, 'sf': 0, 'sa': 0.0005}

plt.figure(figsize=(9, 6))

for ft in color_map.keys():
    indices = [i for i, x in enumerate(file_types) if x == ft]
    x_values = np.array(real_radii)[indices] + shifts[ft]
    y_values = np.array(estimated_radii)[indices] - np.array(real_radii)[indices]
    y_errors = np.array(uncertainties)[indices]

    # Plot della linea sottile e tratteggiata
    plt.plot(x_values, y_values, color=color_map[ft][0], marker='.', linestyle='--', linewidth=0.5, label=color_map[ft][1])

    # Aggiungi barre di errore grosse e semi-trasparenti
    plt.errorbar(x_values, y_values, yerr=y_errors, fmt='s', color=color_map[ft][0], ecolor=color_map[ft][0], elinewidth=3, capsize=8, alpha=0.5)

plt.axhline(0, color='y', linestyle='--', linewidth=1.0)
plt.xlabel('Reference radius [m]')
plt.ylabel('Offset/systematic error [m]')
plt.title('Random and systematic components error of the measurements')
plt.legend(title=r'95% C.I. $\sigma$:')
plt.grid(True)

plt.tight_layout()
plt.show()


















# Creiamo una palette di colori per il tipo di file con nomi personalizzati
palette = {'sv': ('red', 'Viametris'), 'sf': ('blue', 'KA double side'), 'sa': ('green', 'KA single view')}

# Creiamo i subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

# Creazione di una mappa di colori per i file types
colors = [palette[ft][0] for ft in file_types]

# Plot della sfericità con linee di collegamento
for file_type in palette.keys():
    indices = [i for i, ft in enumerate(file_types) if ft == file_type]
    ax1.plot([real_radii[i] for i in indices], [sphericity[i] for i in indices], color=palette[file_type][0], marker='o', label=palette[file_type][1])

ax1.set_xlabel('Reference radius [m]')
ax1.set_ylabel('Sphericity')
ax1.set_title('Sphericity vs Real Radius')
ax1.legend(title='Technique:')
ax1.grid(True)

# Plot della densità con linee di collegamento
for file_type in palette.keys():
    indices = [i for i, ft in enumerate(file_types) if ft == file_type]
    ax2.plot([real_radii[i] for i in indices], [densities[i] for i in indices], color=palette[file_type][0], marker='o', label=palette[file_type][1])

ax2.set_xlabel('Reference radius [m]')
ax2.set_ylabel('Density [points/\n$m^3$]')
ax2.set_title('Density vs Real Radius')
ax2.legend(title='Technique:')
ax2.grid(True)

plt.tight_layout()
plt.show()