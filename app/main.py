import configuration_handle as CONFIG
import ctypes
from ctypes.util import find_library
import sys
import os #for ubuntu only
import time
from collections import deque
from threading import Lock


#---------Ubuntu may need to set up the pacakge location for XDA and keyboard-----#
module_path = "/home/usuario/.local/lib/python3.8/site-packages/"
sys.path.insert(0, module_path)
import xsensdeviceapi.xsensdeviceapi_py38_64 as xda
#---------------------------------------------------------------------------------#




print("config files:",CONFIG.config_file_path)
print(CONFIG.config_data_localization)
PRINT_IMU_RAW = int(CONFIG.config_data_localization['IMU']['print_raw'])
print(PRINT_IMU_RAW)



def inizialize_IMU():
    print("inizializing imu ")

    # Carica le librerie Xsens
    libxsensdeviceapi = ctypes.CDLL(find_library('xsensdeviceapi'))
    libxstypes = ctypes.CDLL(find_library('xstypes'))

    # Inizializza le librerie
    def main():
        version = libxstypes.xsensGetLibraryVersion()
        print(f"Xsens Library Version: {version}")

    if __name__ == "__main__":
        main()


#inizialize_IMU()