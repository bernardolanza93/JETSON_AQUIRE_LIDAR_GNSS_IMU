import serial
import os
import time


def create_path_and_file(directory_path, file_name):
    # Verifica se la directory esiste, altrimenti crea la directory
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)

    # Crea il percorso completo al file
    file_path = os.path.join(directory_path, file_name)

    # Verifica se il file esiste, altrimenti crea il file
    if not os.path.exists(file_path):
        with open(file_path, 'w') as f:
            pass  # Crea un file vuoto

    return file_path


# Definition of the ReadGNSS class
class ReadGNSS:
    # Initialization method for the class
    def __init__(self):
        # Variable initialization to store the GGA line
        self.GGA_line = ''
        self.RMC_line = ''
        # Initialization of the ID to keep track of GNSS readings
        self.gnss_read_ID = 0





        try:
            # Attempt to establish a serial connection to '/dev/ttyACM0' with a baud rate of 115200
            self.ser = serial.Serial('/dev/ttyACM0', 115200)
        except Exception as e:
            print("no ttyACM0:", e)
            try:
                # Attempt to establish a serial connection to '/dev/ttyACM1' with a baud rate of 115200
                self.ser = serial.Serial('/dev/ttyACM1', 115200)
            except Exception as e:
                print(e)
                try:
                    # Attempt to establish a serial connection to 'COM1' with a baud rate of 115200
                    self.ser = serial.Serial('COM1', 115200)
                except Exception as e:
                    print(e)
                    try:
                        # Attempt to establish a serial connection to 'COM2' with a baud rate of 115200
                        self.ser = serial.Serial('COM2', 115200)
                    except Exception as e:
                        print(e)
                        try:
                            # Attempt to establish a serial connection to 'COM3' with a baud rate of 115200
                            self.ser = serial.Serial('COM3', 115200)
                        except Exception as e:
                            print(e)
                            try:
                                # Attempt to establish a serial connection to 'COM4' with a baud rate of 115200
                                self.ser = serial.Serial('COM4', 115200)
                            except Exception as e:
                                print(e)
                                try:
                                    # Attempt to establish a serial connection to 'COM5' with a baud rate of 115200
                                    self.ser = serial.Serial('COM5', 115200)
                                except Exception as e:
                                    print(e)
                                    try:
                                        # Attempt to establish a serial connection to 'COM6' with a baud rate of 115200
                                        self.ser = serial.Serial('COM6', 115200)
                                    except Exception as e:
                                        print(e)
                                        try:
                                            # Attempt to establish a serial connection to 'COM7' with a baud rate of 115200
                                            self.ser = serial.Serial('COM7', 115200)
                                        except Exception as e:
                                            print(e)
                                            # Print error message if Ardusimple is not available and exit
                                            print("Error. Ardusimple not available!")

                                            exit(1)

    # Method to read RMC line from GNSS
    def read_rmc_line(self):
        while 1:
            # Read line from serial and decode it to ASCII
            line = self.ser.readline().decode('ascii', errors='replace')
            # Check if the line contains "RMC"
            if "RMC" in line:
                break
        return line

    # Method to read GGA line from GNSS
    def read_gga_line(self):
        # Close the current serial connection
        self.ser.close()
        # Open the serial connection again
        self.ser.open()
        while 1:
            # Read line from serial and decode it to ASCII
            line = self.ser.readline().decode('ascii', errors='replace')
            # Check if the line contains "GGA"
            if "GGA" in line:
                break
        return line

    # Method to save GGA track data to a file __________________________________DEPRECATED
    def save_gga_track(self, track_file):
        # Open the specified file in append mode
        #control if a file is aviable:



        f1 = open(track_file, 'a')
        while 1:
            try:
                # Read line from serial and decode it to ASCII
                line = self.ser.readline().decode('ascii', errors='replace')
                # Check if the line contains "GGA"
                if "GGA" in line:
                    # Update the GGA line and increment the read ID
                    self.GGA_line = line
                    self.gnss_read_ID += 1
                    # Get the current timestamp
                    timestamp = time.time()
                    # Print the GNSS data read and write it to the file with the timestamp
                    print('\n' + 'GNSS read: ' + str(self.gnss_read_ID) + ' ; ' + line[:-2])
                    f1.write(line[:-2] + ',' + str(timestamp) + '\r\n')
            except KeyboardInterrupt:
                # Break the loop if interrupted by the user
                break
        # Close the file
        f1.close()


#
#
# def record(status,GNSS_queue):
#     while True:
#
#         internal_status = status.value
#
#         if not GNSS_OFFLINE_DEBUG:
#             gnss_reader = ReadGNSS()
#         step_acquisition = 0
#
#         print("WAIT GNSS LOOP ")
#
#         while internal_status == 0:
#             internal_status = status.value
#             time.sleep(0.5)
#
#         print("GNSS START ACQUIRING")
#
#         while internal_status == 1:
#             time.sleep(1)
#
#
#             step_acquisition += 1
#
#             if not GNSS_OFFLINE_DEBUG:
#                 rmc_line = gnss_reader.read_rmc_line()
#                 gnss_reader.RMC_line = rmc_line
#             else:
#                 rmc_line = "sample GNSS data"
#
#
#             timestamp = time.time()
#
#             GNSS_raw_data = [step_acquisition,timestamp,rmc_line]
#
#             try:
#                 GNSS_queue.put(GNSS_raw_data)
#
#             except Exception as e :
#                 print("ERROR QUEUE SENDER GNSS:",e)
#
#
#
#
#
# def data_saver(status,GNSS_queue):
#     while 1:
#         time.sleep(0.5)
#         # organize_video_from_last_acquisition()
#         internal_status = status.value
#
#         print("WAIT SAVER LOOP ")
#
#         while internal_status == 0:
#             internal_status = status.value
#             time.sleep(0.5)
#
#         print("GNSS SAVER READY!,  local_status:", internal_status)
#
#
#         #CREATION DATA FILE
#         now = datetime.now()
#         file_timestamp = now.strftime("%H%M%S%f")[:-3]
#         gnss_data_filename = file_timestamp + gnss_data_identifier_string
#         #path_to_gnss_file = TRACK_FOLDER_GNSS_PATH + gnss_data_filename
#
#         # Crea la directory e il file se non esistono
#         path_to_gnss_file = create_path_and_file(TRACK_FOLDER_GNSS_PATH, gnss_data_filename)
#         print("created GNSS FILE:",path_to_gnss_file)
#
#         with open(path_to_gnss_file, 'a') as f:
#
#
#             while internal_status == 1 or GNSS_queue.qsize() > 0:
#
#                 #qsize = GNSS_queue.qsize()
#                 try:
#                     GNSS_data = GNSS_queue.get()
#                     print(GNSS_data)
#                 except Exception as e:
#                     print("ERROR RECIVER GNSS:",e)
#
#                 f.write(str(GNSS_data) + '\n')  # Aggiungi un newline dopo ogni riga
#
#
#         print("_SAVER_GNSS_ENDED RECORDING_ // FILE SAVED")
#

def simple_GNSS_shower():


    gnss_reader = ReadGNSS()


    while True:




        print("GNSS START ACQUIRING")




        rmc_line = gnss_reader.read_rmc_line()
        gnss_reader.RMC_line = rmc_line


        timestamp = time.time()

        GNSS_raw_data = [timestamp, rmc_line]

        print(GNSS_raw_data)

