

from external_lis import *


def remove_redundant_points(pointcloud, min_distance=0.001):
    """
    Rimuove i punti ridondanti in una nuvola di punti basandosi su una distanza minima,
    preservando i colori e calcolando la percentuale di punti rimossi.

    Parametri:
        pointcloud (o3d.geometry.PointCloud): La nuvola di punti originale.
        min_distance (float): La distanza minima tra punti per considerarli unici.

    Ritorna:
        o3d.geometry.PointCloud: La nuvola di punti filtrata.
        float: Percentuale di punti rimossi.
    """
    # Converti i punti e i colori in array numpy
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors) if pointcloud.has_colors() else None

    # Costruisci un modello di vicinanza usando NearestNeighbors
    nbrs = NearestNeighbors(metric='euclidean', algorithm='auto').fit(points)
    distances, indices = nbrs.kneighbors(points, n_neighbors=2)

    # Identifica gli indici dei punti da mantenere
    unique_indices = set()
    for idx, dist in enumerate(distances):
        if dist[1] > min_distance:
            unique_indices.add(idx)

    # Calcola il numero di punti prima e dopo il filtro
    original_count = len(points)
    filtered_count = len(unique_indices)
    removed_count = original_count - filtered_count

    # Calcola la percentuale di punti rimossi
    removal_percentage = (removed_count / original_count) * 100

    # Mantieni solo i punti e i colori unici
    filtered_points = points[list(unique_indices)]
    filtered_colors = colors[list(unique_indices)] if colors is not None else None

    # Crea una nuova nuvola di punti con i punti e i colori filtrati
    filtered_pointcloud = o3d.geometry.PointCloud()
    filtered_pointcloud.points = o3d.utility.Vector3dVector(filtered_points)
    if filtered_colors is not None:
        filtered_pointcloud.colors = o3d.utility.Vector3dVector(filtered_colors)

    # Stampa informazioni
    #print(f"REDUNDANT Removed: {removed_count} ({removal_percentage:.2f}%)")

    return filtered_pointcloud


def downsample_point_cloud(point_cloud, voxel_size):
    """
    Downsample a point cloud using a voxel grid filter and remove duplicated points.

    Parameters:
        point_cloud (o3d.geometry.PointCloud): The input point cloud.
        voxel_size (float): The voxel size for downsampling.

    Returns:
        o3d.geometry.PointCloud: The downsampled point cloud.
    """

    # Downsample the point cloud using voxel grid filter
    downsampled_pc = point_cloud.voxel_down_sample(voxel_size)

    # Remove duplicate points
    downsampled_pc = downsampled_pc.remove_duplicated_points()

    return downsampled_pc

def remove_isolated_points(pcd, nb_neighbors=5, radius=0.5):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned


def filter_points_within_radius(pointcloud, center, radius):
    #provvisorio solo per l icp, poi vengono recuperati anche i punti esterni
    points = np.asarray(pointcloud.points)
    distances = np.linalg.norm(points - center, axis=1)
    filtered_points = points[distances <= radius]
    filtered_pointcloud = o3d.geometry.PointCloud()
    filtered_pointcloud.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_pointcloud, len(points) - len(filtered_points)

