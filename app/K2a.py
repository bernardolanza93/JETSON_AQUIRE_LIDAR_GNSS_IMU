 import datetime
from pyk4a import Config, ImageFormat, PyK4A, PyK4ARecord, connected_device_count


def initialize_sensor(path, target_serial):
    """
    Initialize the depth sensor.

    Args:
        path (str): The directory where the .mkv file will be stored.
        target_serial (str): The serial number of the Kinect Azure device.

    Returns:
        device: Instance of the PyK4A object, which corresponds to the Kinect Azure.
        record: Instance of the PyK4ARecord object.
    """

    global record, device

    connected_devices = connected_device_count()

    for device_id in range(connected_devices):
        config = Config(color_format=ImageFormat.COLOR_MJPG)
        current_device = PyK4A(device_id=device_id, config=config)
        current_device.open()
        if current_device.serial == target_serial:
            device = current_device
            device.start()

            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = path + "/" + timestamp + ".mkv"

            record = PyK4ARecord(device=device, config=config, path=filename)
            record.create()

            print(f"Kinect Azure {target_serial} init: OK")
            return device, record

        current_device.close()

    print(f"Sensor KA {target_serial} NO trobat.")
    return None, None


def capture(device, record):
    """
    Capture data from the sensor.

    Args:
        device: Instance of the PyK4A object.
        record: Instance of the PyK4ARecord object.
    """
    capture = device.get_capture()
    record.write_capture(capture)


def stop_capture(record):
    """
   Terminate the data capture by closing the record.

   Args:
       record: Instance of the PyK4ARecord object.
   """
    record.flush()
    record.close()


def main():
    """ Main function, just for testing, can be removed """

    ka_path = r'C:/Users/Marc/OneDrive/Documentos/MarcFelip-DigiFruit/CODE/records/KA1'
    device, record = initialize_sensor(ka_path, '000090925112')

    while True:
        capture(device, record)

    stop_capture(record)

    print(f"{record.captures_count} frames written.")


if __name__ == '__main__':
    main()