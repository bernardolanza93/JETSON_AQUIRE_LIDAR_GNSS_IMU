import subprocess

# Definisci il percorso della cartella contenente gli script Bash
cartella = "/sh_node_ros/"

# Definisci il percorso completo degli script Bash
script1 = cartella + "ros_imu_config.sh"
script2 = cartella + "ros_node_pub.sh"

# Avvia entrambi gli script Bash contemporaneamente
process1 = subprocess.Popen(["bash", script1])
process2 = subprocess.Popen(["bash", script2])


# Attendi che entrambi i processi completino l'esecuzione
process1.wait()
process2.wait()

print("Entrambi gli script sono stati eseguiti.")