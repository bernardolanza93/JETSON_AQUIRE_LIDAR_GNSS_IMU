import configuration_handle as CONFIG
import ctypes
from ctypes.util import find_library



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


inizialize_IMU()