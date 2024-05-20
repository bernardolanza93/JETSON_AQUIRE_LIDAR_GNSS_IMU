
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rosbag
from sensor_msgs.msg import Imu
import os
import sys
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np

print("Versione di Python:", sys.version)


def convert_point_cloud2_to_open3d(msg):
    # Estrae i punti dal PointCloud2
    points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    # Converte i punti in un array NumPy
    np_points = np.array(points, dtype=np.float32)

    # Crea un PointCloud di Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)

    return pcd



root_directory = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU"
bag_directory = os.path.join(root_directory, "data")
extension = ".bag"

for root, dirs, files in os.walk(bag_directory):
    for filename in files:
        if filename.endswith(extension):
            print(filename)

            #try:




bag_file = "/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/data/zed_20240517_150359.bag"


###### IMU DATA _________________________________________________________________
# # Apre il file bag
# with rosbag.Bag(bag_file, 'r') as bag:
#     for topic, msg, t in bag.read_messages(topics=['/zedxm/zed_node/imu/data']):
#         print(msg, t)
#
#         if isinstance(msg, Imu):
#             # Qui puoi elaborare i dati dell'IMU
#             print(f"Timestamp: {t.to_sec()}")
#             print(f"Orientation: {msg.orientation}")
#             print(f"Linear acceleration: {msg.linear_acceleration}")
#             print(f"Angular velocity: {msg.angular_velocity}")
##### OPENCV IMAGE STREAM__________________________________________
# # Inizializza CvBridge
# bridge = CvBridge()
#
# # Apre il file bag
# with rosbag.Bag(bag_file, 'r') as bag:
#     for topic, msg, t in bag.read_messages(topics=['/zedxm/zed_node/left_raw/image_raw_color']):
#         print(f"Reading message from topic {topic} at time {t.to_sec()}")
#
#         try:
#             # Converte il messaggio ROS in immagine OpenCV
#             cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#
#             # Visualizza l'immagine utilizzando OpenCV
#             cv2.imshow("Image", cv_image)
#             cv2.waitKey(0)  # Attende un millisecondo per l'aggiornamento della finestra
#
#             # Stampa di debug
#             print(f"Image at time {t.to_sec()} displayed.")
#
#         except CvBridgeError as e:
#             print(f"Error converting image: {e}")
#
# # Chiude tutte le finestre quando il programma termina
# cv2.destroyAllWindows()


# Inizializza una lista per memorizzare i point cloud
pcd_list = []

# Apre il file bag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/zedxm/zed_node/point_cloud/cloud_registered']):
        print(f"Reading message from topic {topic} at time {t.to_sec()}")

        # Converte e aggiunge il point cloud alla lista
        pcd = convert_point_cloud2_to_open3d(msg)
        pcd_list.append(pcd)

        # Stampa di debug
        print(f"Point cloud at time {t.to_sec()} added with {len(pcd.points)} points.")

# Visualizza il primo point cloud usando Open3D
if pcd_list:
    o3d.visualization.draw_geometries([pcd_list[0]])
else:
    print("No point clouds were found in the specified bag file.")