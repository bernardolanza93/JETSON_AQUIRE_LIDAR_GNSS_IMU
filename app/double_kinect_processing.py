from custom_slam_doc import *
from pyk4a import PyK4APlayback, Config, PyK4A
from pyk4a import PyK4APlayback, CalibrationType
import open3d as o3d
import numpy as np
import os
import pandas as pd
import cv2
import matplotlib
import ctypes
import pyk4a
from matrix_utilities import *
import PREPROC_double_KA as KA_PREPROC
import re
import sys


def convert_to_meters(pcd):
    points = np.asarray(pcd.points) / 1000.0  # Convert mm to meters
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def remove_isolated_points(pcd, nb_neighbors=10, radius=500):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_neighbors, radius=radius)
    pcd_cleaned = pcd.select_by_index(ind)
    return pcd_cleaned


def hierarchy_slam_icp(input_all_pointcloud): #100

    # devo saltare la zero che è solo target
    # prendo la sua trasformata trovata tra le sotto pc e la applico alla coppia dopo
    # current_source.trasform(last_epoch_trasformation[i-1])
    # NDR la trasformazione [i-esima] l ho gia usata l epoca prima per traslare la PC
    # tecnicamente cosi ho PC 0,1,2,3  0-1 2-3 gia fuse, applicando questa nuova tresformazione in pratica sto sovrapponendo pc2 a pc1 cosi avremo in serie 0 1+2 e 3
    # la trasformazione nuova mi farà avanzare 2-3 alla fine di 1,
    # ogni iterazione individui la trasformazione 1-2 che però potrebbe crescere con le iterazioni (dipende na quante epoche hanno costruito 1)
    # se volessi avvicinarle ancora dovrei sommare la trasformazione 2-3 anche, (da rivedere perche potrebbe essere troppo)
    # fuse PC[0],PC[1]
    # fuse PC[i],PC[i+1]
    # trasformation_estimated = icp
    # trasfoirmation_current.append(trasformation_estimated)
    # source-trasform to match target ->(trasformation_estimated)
    # fuse target and source transformed
    # new_matrices.append(fusion)

    # al primo giro prendo PC grezze
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0

    last_epoch_trasformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]

    while len(epoch_start_pointclouds) > 1:
        epoch += 1


        i = 1


        dict_analysis ={}

        new_halfed_pointcloud = []
        trasformation_current = []

        print(f"START Epoch {epoch}, PCs:{len(epoch_start_pointclouds)}")


        while i < len(epoch_start_pointclouds):


            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(len(last_epoch_trasformation))
                print(len(epoch_start_pointclouds))

            #TOGLILO
            if i > 0:








                initial_trasform_from_last_epoch_1_couple = (last_epoch_trasformation[i-1])
                initial_trasform_from_last_epoch_2_couple = (last_epoch_trasformation[i])

                # CONSIDERO I PRIOR DI ENTRAMBE LE SOURCE PASSATE, PERCHE SE NO PIU AUMENTANO LE EPOCE PIU AUMENTA IL LAG TRA LE POINTCLOUS.
                # QUINDI PRENDO IL PRIOR TRASFORMATION DA ENTRAMBE LE PC CHE HANNO COSTRUITO LA CORRENTE.

                prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)

                source_raw = epoch_start_pointclouds[i]
                # devo aggiungere anche la trasformata delle iterazioni prima BASTA CHE LE LAST EPOCH LA COMPRENDANO


                current_source = o3d.geometry.PointCloud(source_raw)
                current_source.transform(prior_trasformation_composed)

                target =  epoch_start_pointclouds[i-1]
                updated_trasform_icp_result = icp_open3d(current_source, target)

                #Total trasformation : current, plus the prrevious frame trasf:
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)


                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                merged_pcd = target + trasformed_icp_source
                trasformation_current.append(total_trasformation_prior_plus_icp)
                new_halfed_pointcloud.append(merged_pcd)

                # i = 97, len = 99,
                # quando i = 99 rompe e quindi perderò la pointclou i[98], che sarabbe il target di i[99] che non arriveà
                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:  # Vogliamo eliminare
                # SKip the odd last matrixprint
                   # 100/101 i  = 100

                    print("DISPARI")
                    print(f"START Epoch {epoch},i = {i}, PCs AVIABLE:{len(epoch_start_pointclouds)} PCS comp:{len(new_halfed_pointcloud)} trasform:{len(trasformation_current)}")
                    print("add last PC with its trasformation")
                    # TO DO TO INCLUDE LAST
                    last_pc = epoch_start_pointclouds[i+1] #98
                    last_trasaform = (last_epoch_trasformation[i+1]) #98

                    trasformation_current.append(last_trasaform)
                    new_halfed_pointcloud.append(last_pc)

            i += 2


        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_trasformation = trasformation_current
        print(f"Computed Epoch {epoch}, PCs created:{len(new_halfed_pointcloud)}")




        dict_analysis[epoch] = [trasformation_current , new_halfed_pointcloud]


    return new_halfed_pointcloud[0]

def get_timestamp(file_name):
    timestamp_str = file_name.split('_')[1].split('.ply')[0]

    return float(timestamp_str)



if __name__ == "__main__":

    input_file_mkv_1 =""
    input_file_mkv_2= ""
    output_folder_pc_1 = ""
    output_folder_pc_2 = ""
    start_index_1 = 0
    start_index_2 = 0
    timestamp_conversion_file_1 = ""
    timestamp_conversion_file_2 = ""


    KA_PREPROC.process_mkv(input_file_mkv_1,output_folder_pc_1,start_index_1,timestamp_conversion_file_1)
    KA_PREPROC.process_mkv(input_file_mkv_2,output_folder_pc_2,start_index_2,timestamp_conversion_file_2)

    # TODO check if there is pc file in the destination folder
    # optionally extract and check the trajectory from gnss/imu fused (plot)
    # the goal is to have a fused traj interpolated with pointcloud timestamps



    pointcloud_dir = '/home/mmt-ben/JETSON_AQUIRE_LIDAR_GNSS_IMU/app/pc_ak/pc'
    pointcloud_files = [f for f in os.listdir(pointcloud_dir) if f.endswith('.ply')]
    pointcloud_timestamps = []
    for file in pointcloud_files:
        match = re.search(r'(\d+\.\d+)', file)
        if match:
            timestamp = float(match.group(1))
            pointcloud_timestamps.append(timestamp)

    # Sort files and timestamps together
    pointcloud_files, pointcloud_timestamps = zip(*sorted(zip(pointcloud_files, pointcloud_timestamps)))
    pointcloud_files_sorted = sorted(pointcloud_files, key=get_timestamp)
    start = 2297
    timestamp_sorted = []
    pointclouds = []
    for idx, file_name in enumerate(pointcloud_files_sorted):
        timestamp_str = file_name.split('_')[1].split('.ply')[0]
        timestamp = float(timestamp_str)
        timestamp_sorted.append(timestamp)

        pcd_raw = o3d.io.read_point_cloud(os.path.join(pointcloud_dir, file_name))  # Usa file_name invece di pointcloud_files[idx]
        pcd = remove_isolated_points(pcd_raw)
        pcd_m = convert_to_meters(pcd)
        pointclouds.append(pcd_m)




    # define the initial trasformation between the upper and lower kinect
    # zippa le coppie di pointcloud basandoti sul loro timestamp
    # TODO take pairs of pointcloud from two folder, trasform and register in accrod wwith the rigid trasformation
    # and then peorform ICP to fuse upper and lower PC
    # salvale in una terzo folder da cui poi le ripescherai

    fused = hierarchy_slam_icp(pointclouds)
    save_pointcloud(fused, "fusedrow.ply")

    point_size = 1
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Merged Point Cloud A')

    # Aggiungi le geometrie una alla volta
    vis.add_geometry(fused)


    opt = vis.get_render_option()
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()

    sys.exit()
