import configuration_handle as CONFIG
import GNSS_utility as GNSS

import sys
import os #for ubuntu only
import time
from collections import deque
from threading import Lock


#---------------------------------------------------------------------------------#




print("config files:",CONFIG.config_file_path)
print(CONFIG.config_data_localization)
PRINT_IMU_RAW = int(CONFIG.config_data_localization['IMU']['print_raw'])
print(PRINT_IMU_RAW)


GNSS.simple_GNSS_shower()