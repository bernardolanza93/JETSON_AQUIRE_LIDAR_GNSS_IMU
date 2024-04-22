import multiprocessing
import time
import os
import serial
import configparser
from datetime import datetime
import sys
import keyboard



GNSS_OFFLINE_DEBUG = 1
gnss_data_identifier_string = "_GNSS_DATA.txt"



import configuration_handle as CONFIG
print("config files:",CONFIG.config_file_path)
print("config DATA:",CONFIG.config_data_localization)
TRACK_FOLDER_GNSS_PATH = CONFIG.config_data_localization['GNSS']['track_folder']
print("GNSS DATA FOLDER:",TRACK_FOLDER_GNSS_PATH)
PRINT_IMU_RAW = int(CONFIG.config_data_localization['IMU']['print_raw'])
print("IMU PRINT ON TERMINAL:",PRINT_IMU_RAW)