import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import time

class BarcoControl(Node):
    def __init__(self):
        super().__init__('barco_control')
        
        self.get_logger().info('🚤 PROYECTO IVAN - Modo WiFi Cliente')
        
        # 🔄 CAMBIA ESTA IP POR LA QUE TE DÉ EL ESP32
        self.esp32_ip = "192.168.4.1"  # ← ESTA IP CAMBIA! Anota la que muestre el ESP32
        self.esp32_port = 8888
        
        # Conectar por WiFi
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.esp32_ip, self.esp32_port))
            self.sock.settimeout(1)
            self.get_logger().info(f'✅ Conectado por WiFi a {self.esp32_ip}:{self.esp32_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Error WiFi: {e}')
            self.sock = None
        
        # Escuchar joystick
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.ultimo_comando = 'S'
        self.get_logger().info('🎮 IVAN WiFi listo!')
    
    def joy_callback(self, msg):
        comando = 'S'
        
        if msg.axes[1] < -0.5:    # ARRIBA
            comando = 'F'
        elif msg.axes[1] > 0.5:   # ABAJO
            comando = 'B'
        elif msg.axes[0] < -0.5:  # IZQUIERDA
            comando = 'L'
        elif msg.axes[0] > 0.5:   # DERECHA
            comando = 'R'
        
        if comando != self.ultimo_comando:
            self.enviar_comando_wifi(comando)
            self.ultimo_comando = comando
    
    def enviar_comando_wifi(self, cmd):
        if self.sock:
            try:
                self.sock.send(f'{cmd}\n'.encode())
                self.get_logger().info(f'📡 WiFi -> IVAN: {cmd}')
            except Exception as e:
                self.get_logger().error(f'Error WiFi: {e}')
                # Intentar reconectar
                try:
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.connect((self.esp32_ip, self.esp32_port))
                    self.sock.settimeout(1)
                except:
                    self.sock = None
        else:
            self.get_logger().info(f'[SIM] {cmd}')

def main():
    rclpy.init()
    node = BarcoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Apagando IVAN WiFi...')
    finally:
        if hasattr(node, 'sock') and node.sock:
            node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()