import threading
import time
from datetime import datetime


# Variabile di stato condivisa
stato = 0

# Variabile condivisa per il timestamp
timestamp = 0



# Coda condivisa
coda = []

def thread_produttore():
    global stato, timestamp, coda
    stato_interno = 0

    prodotto = 0

    while True:
        # Controlla lo stato
        with mutex:
            if stato_interno != stato:
                print(timestamp, ": PROD: state change:", stato)

            stato_interno = stato
            if stato_interno == 0:
                prodotto = 0
                # Aggiorna il timestamp
            elif stato_interno == 1:
                prodotto += 1
                coda.append(prodotto)


        # Aspetta il cambio di stato
        time.sleep(0.7)

def thread_consumatore():
    global stato, coda
    stato_interno = 0

    while True:
        # Controlla lo stato
        with mutex:
            if stato_interno != stato:
                print(timestamp, ": CONS: state change:", stato)

            stato_interno = stato
            if stato_interno == 1:
                # Se la coda è vuota, aspetta
                if not coda:
                    continue

                # Consuma un valore dalla coda
                valore = coda.pop(0)
                print(timestamp, ": consumed ", valore)
            elif stato_interno == 0:

                if len(coda) > 0:

                    valore = coda.pop(0)
                    print(timestamp, ": QUEUE EMPTYING, remains: ", len(coda))



        # Aspetta il cambio di stato
        time.sleep(1)

def thread_ascoltatore_tastiera():
    global stato

    while True:




        # Aspetta la pressione di un tasto
        print(".")
        key = input("PRESS 0 or 1: ")
        print(".")

        # Aggiorna lo stato in base al tasto premuto
        if key == "0":
            stato = 0

        elif key == "1":
            stato = 1

def timestampator():
    global stato, timestamp

    while True:
        # Ottieni l'ora corrente
        now = datetime.now()

        # Formatta l'ora in HHMMSS
        inner_timestamp = now.strftime("%H%M%S%f")[:-3]
        timestamp = inner_timestamp







if __name__ == "__main__":
    # Crea il mutex
    mutex = threading.Lock()

    # Crea e avvia i thread
    thread_produttore = threading.Thread(target=thread_produttore)
    thread_consumatore = threading.Thread(target=thread_consumatore)
    thread_ascoltatore_tastiera = threading.Thread(target=thread_ascoltatore_tastiera)
    thread_timestamp = threading.Thread(target=timestampator)

    thread_produttore.start()
    thread_consumatore.start()
    thread_ascoltatore_tastiera.start()
    thread_timestamp.start()
