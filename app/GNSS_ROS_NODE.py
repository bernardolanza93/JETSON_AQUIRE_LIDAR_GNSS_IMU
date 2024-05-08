import rospy
import GNSS_utility as GNSS  # Importa il pacchetto e la classe del sensore

class ROSNode(object):
    def __init__(self):
        # Inizializza il nodo ROS
        self.node_name = "GNSS_NODE"
        rospy.init_node(self.node_name)

        # Crea un publisher per pubblicare i dati del sensore
        self.pub = rospy.Publisher("GNSS_TOPIC", sensor_msgs.msg.SensorData, queue_size=10)

        # Crea un'istanza del sensore
        self.sensor = YourSensor()

        # Imposta la frequenza di pubblicazione
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            # Leggi i dati dal sensore
            sensor_data = self.sensor.read_data()

            # Crea un messaggio SensorData
            sensor_msg = sensor_msgs.msg.SensorData()
            sensor_msg.data = sensor_data

            # Pubblica il messaggio sul topic
            self.pub.publish(sensor_msg)

            # Aspetta la prossima iterazione del loop
            self.rate.sleep()

if __name__ == "__main__":
    node = ROSNode()
    node.run()
