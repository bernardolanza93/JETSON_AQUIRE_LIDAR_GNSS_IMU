import subprocess

# Definisci il percorso della cartella contenente gli script Bash
cartella = "sh_node_ros/"

# Definisci il percorso completo degli script Bash
script1 = cartella + "ros_imu_config.sh"
script2 = cartella + "ros_node_pub.sh"

# Avvia entrambi gli script Bash contemporaneamente
process1 = subprocess.Popen(["bash", script1], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
process2 = subprocess.Popen(["bash", script2], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# Crea un terzo processo per salvare l'output del secondo processo su file
with open("output_script2.txt", "w") as output_file:
    # Leggi l'output del secondo processo e scrivilo su file
    for line in iter(process2.stdout.readline, ''):
        output_file.write(line)
        print(line.strip())  # Stampa l'output per vedere in tempo reale
        if process2.poll() is not None:
            break


# Attendi che entrambi i processi completino l'esecuzione
process1.wait()
process2.wait()

print("Entrambi gli script sono stati eseguiti.")
