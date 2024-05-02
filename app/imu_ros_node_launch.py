import subprocess

# Definisci il percorso della cartella contenente gli script Bash
cartella = "sh_node_ros/"

# Definisci il percorso completo degli script Bash
script1 = cartella + "ros_imu_config.sh"
script2 = cartella + "ros_node_pub.sh"

# Avvia entrambi gli script Bash contemporaneamente
process1 = subprocess.Popen(["bash", script1], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
process2 = subprocess.Popen(["bash", script2], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# Ottieni l'output e l'output di errore dello script 1
output_script1, error_script1 = process1.communicate()
print("Output dello script 1:")
print(output_script1)

# Ottieni l'output e l'output di errore dello script 2
output_script2, error_script2 = process2.communicate()
print("\nOutput dello script 2:")
print(output_script2)

# Stampa eventuali errori
if error_script1:
    print("\nErrore nello script 1:")
    print(error_script1)
if error_script2:
    print("\nErrore nello script 2:")
    print(error_script2)