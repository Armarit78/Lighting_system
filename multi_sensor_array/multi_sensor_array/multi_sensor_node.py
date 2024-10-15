from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32
from rclpy.node import Node
import rclpy


class MultiSensorNode(Node):
    def __init__(self):
        super().__init__('multi_sensor_node')

        # Créer les publishers pour les capteurs de température et d'humidité
        self.temp_pub = self.create_publisher(Temperature, 'temperature_data', 10)
        self.humidity_pub = self.create_publisher(Float32, 'humidity_data', 10)

        # Initialiser les valeurs par défaut (sans timer de publication en continu)
        self.prev_temp = None
        self.prev_humidity = None

        # Subscription aux données venant de l'interface ou autres sources
        self.create_subscription(Float32, 'humidity_control', self.humidity_callback, 10)

    def humidity_callback(self, msg):
        humidity = msg.data

        # Publier seulement si la valeur d'humidité change
        if self.prev_humidity is None or abs(humidity - self.prev_humidity) > 1.0:  # Seuil de 1%
            self.prev_humidity = humidity
            self.humidity_pub.publish(msg)
            self.get_logger().info(f'Humidity set to: {humidity}%')

    def publish_temperature(self, value):
        temp_msg = Temperature()
        temp_msg.temperature = float(value)
        temp_msg.variance = 0.5  # Exemple de variance
        self.temp_pub.publish(temp_msg)
        self.get_logger().info(f'Temperature set to: {value}°C')


def main(args=None):
    rclpy.init(args=args)
    sensor_node = MultiSensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

