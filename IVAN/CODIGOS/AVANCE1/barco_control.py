import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class BarcoControl(Node):
    def __init__(self):
        super().__init__('barco_control')
        
        self.get_logger().info(' PROYECTO IVAN - Iniciando...')
        
        # Conectar con el ESP32
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info(' Conectado al ESP32 en /dev/ttyUSB0 - Barco IVAN listo!')
        except Exception as e:
            self.get_logger().error(f' Error conectando al ESP32: {e}')
            self.get_logger().info(' Ejecutando en modo simulación')
        
        # Escuchar el joystick
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        
        self.ultimo_comando = 'S'
        self.get_logger().info(' Sistema listo. Usa los ejes 6 y 7 para controlar IVAN!')
        self.get_logger().info(' Eje 7: Negativo=Adelante, Positivo=Atrás')
        self.get_logger().info(' Eje 6: Negativo=Izquierda, Positivo=Derecha')
    
    def joy_callback(self, msg):
        """
        msg.axes[6] -> Eje 6 (negativo: izquierda, positivo: derecha)
        msg.axes[7] -> Eje 7 (negativo: adelante, positivo: atrás)
        """
        comando = 'S'  # Por defecto STOP
        
        # LÓGICA DE CONTROL CON EJES 6 Y 7
        if msg.axes[7] < -0.5:    # Eje 7 NEGATIVO - ADELANTE
            comando = 'F'
            self.get_logger().info('IVAN:  ADELANTE (Eje 7 negativo)')
        elif msg.axes[7] > 0.5:   # Eje 7 POSITIVO - ATRÁS
            comando = 'B' 
            self.get_logger().info('IVAN:  ATRÁS (Eje 7 positivo)')
        elif msg.axes[6] < -0.5:  # Eje 6 NEGATIVO - IZQUIERDA
            comando = 'L'
            self.get_logger().info('IVAN:  IZQUIERDA (Eje 6 negativo)')
        elif msg.axes[6] > 0.5:   # Eje 6 POSITIVO - DERECHA
            comando = 'R'
            self.get_logger().info('IVAN:  DERECHA (Eje 6 positivo)')
    
        # Solo enviar si el comando cambió
        if comando != self.ultimo_comando:
            self.enviar_comando(comando)
            self.ultimo_comando = comando
    
    def enviar_comando(self, cmd):
        if self.serial_conn is not None:
            try:
                self.serial_conn.write(f'{cmd}\n'.encode())
                self.get_logger().info(f' Comando enviado a IVAN: {cmd}')
            except Exception as e:
                self.get_logger().error(f' Error enviando comando: {e}')
        else:
            self.get_logger().info(f'[SIMULACIÓN] Comando para IVAN: {cmd}')

def main():
    rclpy.init()
    node = BarcoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' Apagando Barco IVAN...')
    finally:
        if node.serial_conn is not None:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
