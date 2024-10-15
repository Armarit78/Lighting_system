from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Float32
import rclpy
from rclpy.node import Node

class CentralAIHub(Node):
    def __init__(self):
        super().__init__('central_ai_hub')

        # Subscription aux données de température et d'humidité
        self.create_subscription(Temperature, 'temperature_data', self.temperature_callback, 10)
        self.create_subscription(Float32, 'humidity_data', self.humidity_callback, 10)

        # Publishers pour ajuster l'éclairage et contrôler l'humidité
        self.light_color_pub = self.create_publisher(String, 'light_color', 10)
        self.humidity_control_pub = self.create_publisher(String, 'humidity_control', 10)

    def temperature_callback(self, msg):
        temperature = msg.temperature
        # Ajuster la couleur de la lumière en fonction de la température
        if temperature < 18.0:
            color = 'warm'
        else:
            color = 'cool'
        
        light_msg = String()
        light_msg.data = f'Adjusting light color to {color} due to temperature: {temperature}°C'
        self.light_color_pub.publish(light_msg)
        self.get_logger().info(f'Light color adjusted to {color} for temperature: {temperature}°C')

    def humidity_callback(self, msg):
        humidity = msg.data
        # Contrôler l'humidité (par exemple, activer un déshumidificateur si l'humidité est élevée)
        if humidity > 70.0:
            action = 'Dehumidifier ON'
        else:
            action = 'Normal operation'
        
        humidity_msg = String()
        humidity_msg.data = f'{action} due to humidity: {humidity}%'
        self.humidity_control_pub.publish(humidity_msg)
        self.get_logger().info(f'{action} for humidity: {humidity}%')

def main(args=None):
    rclpy.init(args=args)
    ai_hub_node = CentralAIHub()
    rclpy.spin(ai_hub_node)
    ai_hub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
