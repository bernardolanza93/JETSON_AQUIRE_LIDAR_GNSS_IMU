
# This is a file of example to use Zed and Ardusimple GNSS. This version is similar to zed_ardusimple_record_v5. Minor changes has been added in the ReadGNSS class. In the previous version, the "save_gga_track" funciton was called "read_gga_line". This new code has not been tested yet, and the current funtion "read_gga_line" did not exist.


import pyzed.sl as sl
import serial
import multiprocessing
import time

save_directory = '//home/grap/Documents/ZED_ardusimple_records/'

camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1


def main():
    print("Running...")

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    #init.camera_resolution = sl.RESOLUTION.HD720
    # init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = 30
    cam = sl.Camera()
    if not cam.is_opened():
        print("Opening ZED Camera...")
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    runtime = sl.RuntimeParameters()
    mat = sl.Mat()

    record(cam, runtime, mat)
    cam.close()
    print("\nFINISH")



def record(cam, runtime, mat):
    gnss_reader = ReadGNSS()
    rmc_line = gnss_reader.read_rmc_line()
    line_list = rmc_line.split(",")
    track_file = save_directory + '/' + line_list[9][-2:] + line_list[9][2:4] + line_list[9][:2] + '_' + line_list[1][
                                                                                  :-3] + '_GNSS_GGA_track.txt'
    svo_file = save_directory + '/' + line_list[9][-2:] + line_list[9][2:4] + line_list[9][:2] + '_' + line_list[1][
                                                                                  :-3] + '_zed.svo'
    print('Saving GNSS data to: ' + track_file)
    print('Saving ZED data to: ' + svo_file)

    vid = sl.ERROR_CODE.FAILURE
    out = False

    while vid != sl.ERROR_CODE.SUCCESS and not out:
        record_param = sl.RecordingParameters(svo_file)
        vid = cam.enable_recording(record_param)
        print(repr(vid))
        # cv2.destroyAllWindows()
        if vid == sl.ERROR_CODE.SUCCESS:
            print("Recording started...")
            t = multiprocessing.Process(target=gnss_reader.save_gga_track, args=(track_file,))
            t.start()
            step = 0
            out = True
            try:
                while 1:
                    err = cam.grab(runtime)
                    if err == sl.ERROR_CODE.SUCCESS:
                        print('Frame: ' + str(step))
                        step += 1
            except KeyboardInterrupt:
                print("KeyboardInterrupt (Ctrl-C)")
                pass
        else:
            print("Help: you must enter the filepath + filename + SVO extension.")
            print("Recording not started.")
    cam.disable_recording()
    cam.close()
    print("Recording finished.")

class ReadGNSS:
    def __init__(self):
        self.GGA_line = ''  # To store the data of the measurement
        self.gnss_read_ID = 0  # To keep track of the step
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200)
        except:
            try:
                self.ser = serial.Serial('/dev/ttyACM1', 115200)
            except:
                try:
                    self.ser = serial.Serial('COM1', 115200)
                except:
                    try:
                        self.ser = serial.Serial('COM2', 115200)
                    except:
                        try:
                            self.ser = serial.Serial('COM3', 115200)
                        except:
                            try:
                                self.ser = serial.Serial('COM4', 115200)
                            except:
                                try:
                                    self.ser = serial.Serial('COM5', 115200)
                                except:
                                    try:
                                        self.ser = serial.Serial('COM6', 115200)
                                    except:
                                        try:
                                            self.ser = serial.Serial('COM7', 115200)
                                        except:
                                            print("Error. Ardusimple no disponible!")
                                            exit(1)
    def read_rmc_line(self):
        while 1:
            line = self.ser.readline().decode('ascii', errors='replace')
            if "RMC" in line:
                break
        return line

    def read_gga_line(self):
        self.ser.close()
        self.ser.open()
        while 1:
            line = self.ser.readline().decode('ascii', errors='replace')
            if "GGA" in line:
                break
        return line

    def save_gga_track(self, track_file):
        f1 = open(track_file, 'a')
        while 1:
            try:
                line = self.ser.readline().decode('ascii', errors='replace')
                if "GGA" in line:
                    self.GGA_line = line
                    self.gnss_read_ID += 1
                    timestamp = time.time()
                    print('\n' + 'GNSS read: ' + str(self.gnss_read_ID) + ' ; ' + line[:-2])
                    f1.write(line[:-2] + ',' + str(timestamp) + '\r\n')
            except KeyboardInterrupt:
                break
        f1.close()



if __name__ == "__main__":
    main()
