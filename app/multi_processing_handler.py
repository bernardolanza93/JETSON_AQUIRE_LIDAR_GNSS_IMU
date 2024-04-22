from modules import *
import GNSS_utility as GNSS



if __name__ == "__main__":





    process_ids = []
    # Creazione di una variabile multiprocesso condivisa per lo stato
    status = multiprocessing.Value('i', 0)
    GNSS_queue = multiprocessing.Queue()


    # Creation Processes:
    gnss = multiprocessing.Process(target=GNSS.record, args=(status,GNSS_queue))
    gnss_saver = multiprocessing.Process(target=GNSS.data_saver, args=(status,GNSS_queue))
   # listener = multiprocessing.Process(target=input_listener, args=(status,))

    # Avvio dei processi

    gnss.start()
    gnss_saver.start()
    #listener.start()

    process_ids.append(os.getpid())
    process_ids.append(gnss.pid)
    process_ids.append(gnss_saver.pid)

    print("ID of main_mp_handler -> {}".format(os.getpid()))
    print("ID of gnss            -> {}".format(gnss.pid))
    print("ID of gnss_saver      -> {}".format(gnss_saver.pid))

    while input() != 2:

        key = input("PRESS 0 or 1: ")
        print("pressed: ", key)


        if key == "0":

            status.value = 0
        elif key == "1":

            status.value = 1

    gnss.join()
    gnss_saver.join()

    print("TERMINATED_______________________")





