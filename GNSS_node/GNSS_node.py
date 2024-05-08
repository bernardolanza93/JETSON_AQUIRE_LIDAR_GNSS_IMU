#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import time



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




def main():
    rospy.init_node('GNSS_node')
    pub = rospy.Publisher('GNSS', String, queue_size=10)

    # Inizializza la comunicazione seriale qui

    rate = rospy.Rate(10)  # Frequenza di pubblicazione, 10 Hz per esempio
    gnss_reader = ReadGNSS()

    while not rospy.is_shutdown():
        # Leggi i dati dal sensore seriale
        timestamp = time.time()
        rmc_line = gnss_reader.read_rmc_line()
        gnss_reader.RMC_line = rmc_line

        data_pack = str(timestamp) + " _ " + rmc_line
        # Inserisci il codice per leggere i dati dal sensore seriale qui
        rospy.loginfo(data_pack)

        pub.publish(data_pack)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
