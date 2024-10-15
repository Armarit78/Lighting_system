import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from threading import Thread


class SensorControlNode(Node):
    def __init__(self):
        super().__init__('sensor_control_node')
        
        # Création des publishers pour la température et l'humidité
        self.temp_pub = self.create_publisher(Temperature, 'temperature_data', 10)
        self.humidity_pub = self.create_publisher(Float32, 'humidity_data', 10)

    def publish_temperature(self, value):
        # Assurer que la valeur est convertie en float avant de la publier
        temp_msg = Temperature()
        temp_msg.temperature = float(value)  # Conversion explicite en float
        temp_msg.variance = 0.5  # Exemple de variance
        self.temp_pub.publish(temp_msg)
        self.get_logger().info(f'Temperature set to: {value}°C')

    def publish_humidity(self, value):
        # Assurer que la valeur est convertie en float avant de la publier
        humidity_msg = Float32()
        humidity_msg.data = float(value)  # Conversion explicite en float
        self.humidity_pub.publish(humidity_msg)
        self.get_logger().info(f'Humidity set to: {value}%')


class SensorControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node
        self.setWindowTitle('Sensor Control')

        # Layout principal
        layout = QVBoxLayout()

        # Slider pour la température
        self.temp_label = QLabel('Temperature: 22°C')
        self.temp_slider = QSlider(Qt.Horizontal)
        self.temp_slider.setMinimum(0)
        self.temp_slider.setMaximum(40)
        self.temp_slider.setValue(22)
        self.temp_slider.valueChanged.connect(self.update_temperature)
        layout.addWidget(self.temp_label)
        layout.addWidget(self.temp_slider)

        # Slider pour l'humidité
        self.humidity_label = QLabel('Humidity: 45%')
        self.humidity_slider = QSlider(Qt.Horizontal)
        self.humidity_slider.setMinimum(0)
        self.humidity_slider.setMaximum(100)
        self.humidity_slider.setValue(45)
        self.humidity_slider.valueChanged.connect(self.update_humidity)
        layout.addWidget(self.humidity_label)
        layout.addWidget(self.humidity_slider)

        # Définir le layout de la fenêtre
        self.setLayout(layout)

    def update_temperature(self, value):
        self.temp_label.setText(f'Temperature: {value}°C')
        self.node.publish_temperature(value)  # Publier uniquement lorsque le slider change

    def update_humidity(self, value):
        self.humidity_label.setText(f'Humidity: {value}%')
        self.node.publish_humidity(value)  # Publier uniquement lorsque le slider change


def ros_spin_thread(node):
    """Thread séparé pour ROS 2."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)

    # Créer le nœud ROS 2
    node = SensorControlNode()

    # Lancer ROS 2 dans un thread séparé
    ros_thread = Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()

    # Créer l'application PyQt5
    app = QApplication(sys.argv)

    # Créer l'interface graphique et passer le nœud ROS
    gui = SensorControlGUI(node)
    gui.show()

    # Exécuter l'interface graphique
    sys.exit(app.exec_())

    # Détruire le nœud proprement après la fermeture de l'interface graphique
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
