import subprocess

# Definisci il percorso della cartella contenente gli script Bash
cartella = "sh_node_ros/"

# Definisci il percorso completo degli script Bash
script1 = cartella + "ros_imu_config.sh"
script2 = cartella + "ros_node_pub.sh"

# Avvia entrambi gli script Bash contemporaneamente
process1 = subprocess.Popen(["bash", script1], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
process2 = subprocess.Popen(["bash", script2], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

# Leggi e stampa l'output continuo dello script 1
print("Output continuo dello script 1:")
for line in iter(process1.stdout.readline, ''):
    print(line.strip())
    if process1.poll() is not None:
        break

# Leggi e stampa l'output continuo dello script 2
print("\nOutput continuo dello script 2:")
for line in iter(process2.stdout.readline, ''):
    print(line.strip())
    if process2.poll() is not None:
        break

# Attendere che entrambi i processi completino l'esecuzione
process1.wait()
process2.wait()

print("\nEntrambi gli script sono stati eseguiti.")