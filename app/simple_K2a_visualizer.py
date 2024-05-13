import cv2
import k2a

def main():
    # Inizializza il dispositivo Kinect
    device = k2a.Device()

    # Avvia lo streaming dalla Kinect
    device.start_cameras()

    while True:
        # Ottieni i frame RGB e di profondità dalla Kinect
        rgb_frame, depth_frame = device.get_frames()

        # Converte il frame RGB in formato OpenCV BGR
        rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

        # Visualizza il frame RGB
        cv2.imshow('RGB Frame', rgb_frame)

        # Visualizza il frame di profondità (opzionale)
        # cv2.imshow('Depth Frame', depth_frame)

        # Aspetta 1 millisecondo per l'input dell'utente e interrompi se viene premuto 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Ferma lo streaming e rilascia le risorse
    device.stop_cameras()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()