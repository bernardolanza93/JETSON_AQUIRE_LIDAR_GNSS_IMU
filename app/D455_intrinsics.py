import pyrealsense2 as rs
import numpy as np
import cv2

# Contesto e pipeline
ctx = rs.context()
pipeline = rs.pipeline(ctx)
config = rs.config()

device_aviable = {}

# Verifica dispositivi connessi
if len(ctx.devices) > 0:
    for d in ctx.devices:
        device = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        model = str(device.split(' ')[-1])
        device_aviable[model] = [serial, device]
        print('Found device: ', device_aviable)
else:
    print("No Intel Device connected")

# Serial number della D455
seriald455 = str(device_aviable['D455'][0])

# Abilita i flussi
config.enable_device(seriald455)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Allinea i frame di profondità con quelli del colore
align_to = rs.stream.color
align = rs.align(align_to)

pipeline.start(config)

frame_c = 0

def print_intrinsics(intrinsics, name):
    print(f"\n{name} intrinsics:")
    print(f"Width: {intrinsics.width}")
    print(f"Height: {intrinsics.height}")
    print(f"ppx (cx): {intrinsics.ppx}")
    print(f"ppy (cy): {intrinsics.ppy}")
    print(f"fx: {intrinsics.fx}")
    print(f"fy: {intrinsics.fy}")
    print(f"Distortion model: {intrinsics.model}")
    print(f"Distortion coefficients: {intrinsics.coeffs}")

def get_max_resolution(depth_frame):
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    return width, height

try:
    while True:
        frames = pipeline.wait_for_frames()
        frame_c += 1

        aligned_frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Verifica se i frame sono validi
        if not depth_frame or not color_frame:
            print("Frame non validi.")
            continue

        # Salva i parametri intrinseci dei primi frame
        if frame_c < 3:
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

            print_intrinsics(depth_intrin, "DEPTH")
            print_intrinsics(color_intrin, "COLOR")

            # Verifica se i parametri intrinseci sono uguali
            if (depth_intrin.width == color_intrin.width and
                    depth_intrin.height == color_intrin.height and
                    depth_intrin.ppx == color_intrin.ppx and
                    depth_intrin.ppy == color_intrin.ppy and
                    depth_intrin.fx == color_intrin.fx and
                    depth_intrin.fy == color_intrin.fy):
                print("ATTENZIONE: I parametri intrinseci di profondità e colore sono uguali, verifica il codice.")
            else:
                print("I parametri intrinseci di profondità e colore sono diversi, il che è corretto.")

        # Visualizza il frame di profondità
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Calcola la risoluzione massima del frame di profondità
        max_res = get_max_resolution(depth_frame)
        print(f"Max resolution of depth frame: {max_res}")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
