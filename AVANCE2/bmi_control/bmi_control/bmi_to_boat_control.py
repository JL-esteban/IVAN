#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import time

class BMIToBoatControl(Node):
    def __init__(self):
        super().__init__('bmi_to_boat_control')
        
        self.get_logger().info('🚤 PROYECTO IVAN - Control BMI160 → Barco WiFi')
        
        # 📝 Parámetros configurables
        self.declare_parameter('esp32_ip', '192.168.4.1')
        self.declare_parameter('esp32_port', 8888)
        self.declare_parameter('threshold', 0.3)  # Umbral para detectar movimiento
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = self.get_parameter('esp32_port').value
        self.threshold = self.get_parameter('threshold').value
        
        # Conectar por WiFi al ESP32
        self.sock = None
        self.conectar_wifi()
        
        # Suscribirse al topic del BMI160
        self.subscription = self.create_subscription(
            Twist,
            'bmi_joystick',
            self.bmi_callback,
            10
        )
        
        self.ultimo_comando = 'S'
        self.get_logger().info(f'🎮 Escuchando topic: bmi_joystick')
        self.get_logger().info(f'📡 WiFi configurado: {self.esp32_ip}:{self.esp32_port}')
        self.get_logger().info(f'⚙️  Umbral de movimiento: ±{self.threshold}')
    
    def conectar_wifi(self):
        """Conecta al ESP32 por WiFi"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.esp32_ip, self.esp32_port))
            self.sock.settimeout(1)
            self.get_logger().info(f'✅ Conectado por WiFi a {self.esp32_ip}:{self.esp32_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Error WiFi: {e}')
            self.get_logger().warning('🔄 Continuando en modo simulación')
            self.sock = None
    
    def bmi_callback(self, msg):
        """
        Recibe datos del BMI160 y los convierte a comandos
        msg.linear.x  -> Adelante/Atrás (Y del joystick: -1.0 a 1.0)
        msg.angular.z -> Izquierda/Derecha (X del joystick: -1.0 a 1.0)
        """
        
        # Priorizar movimiento lineal sobre angular
        comando = 'S'  # Por defecto: Stop
        
        # Adelante/Atrás tiene prioridad
        if msg.linear.x > self.threshold:
            comando = 'F'  # Adelante
        elif msg.linear.x < -self.threshold:
            comando = 'B'  # Atrás
        # Si no hay movimiento lineal significativo, revisar giros
        elif msg.angular.z > self.threshold:
            comando = 'L'  # Izquierda
        elif msg.angular.z < -self.threshold:
            comando = 'R'  # Derecha
        
        # Solo enviar si el comando cambió
        if comando != self.ultimo_comando:
            self.enviar_comando_wifi(comando)
            self.ultimo_comando = comando
            
            # Log con valores del joystick
            self.get_logger().info(
                f'🎯 BMI → {comando} | '
                f'Linear.x: {msg.linear.x:+.2f} Angular.z: {msg.angular.z:+.2f}'
            )
    
    def enviar_comando_wifi(self, cmd):
        """Envía comando al ESP32 por WiFi"""
        if self.sock:
            try:
                self.sock.send(f'{cmd}\n'.encode())
                self.get_logger().info(f'📡 WiFi → IVAN: {cmd}')
            except Exception as e:
                self.get_logger().error(f'⚠️  Error enviando: {e}')
                # Intentar reconectar
                self.get_logger().info('🔄 Intentando reconectar...')
                try:
                    if self.sock:
                        self.sock.close()
                    self.conectar_wifi()
                except:
                    self.sock = None
        else:
            # Modo simulación (sin conexión WiFi)
            self.get_logger().info(f'[SIM] 🎮 Comando: {cmd}')
    
    def destroy_node(self):
        """Limpieza al cerrar el nodo"""
        if self.sock:
            try:
                self.sock.close()
                self.get_logger().info('🔌 Conexión WiFi cerrada')
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BMIToBoatControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⏹️  Apagando control BMI...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
