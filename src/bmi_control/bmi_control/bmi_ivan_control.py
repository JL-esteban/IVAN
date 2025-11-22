#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import serial
import time

class BMIIvanControl(Node):
    def __init__(self):
        super().__init__('bmi_ivan_control')
        
        self.get_logger().info(' PROYECTO IVAN - Control con BMI160')
        
        # Configuración WiFi para ESP32
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 8888
        
        # Conectar WiFi
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.esp32_ip, self.esp32_port))
            self.sock.settimeout(1)
            self.get_logger().info(f' WiFi conectado: {self.esp32_ip}:{self.esp32_port}')
        except Exception as e:
            self.get_logger().error(f' Error WiFi: {e}')
            self.sock = None
        
        # Conectar Arduino BMI160
        puerto_serial = '/dev/ttyACM0'  # Cambia si es necesario
        try:
            self.ser = serial.Serial(puerto_serial, 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info(f' Arduino BMI160 conectado: {puerto_serial}')
        except Exception as e:
            self.get_logger().error(f' Error Arduino: {e}')
            self.get_logger().error('Puertos: /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyACM1')
            self.ser = None
            return
        
        # Timer para leer BMI (20Hz)
        self.timer = self.create_timer(0.05, self.leer_bmi_y_controlar)
        
        self.ultimo_comando = 'S'
        self.get_logger().info('Sistema listo! Mueve el BMI160 para controlar IVAN')
        self.get_logger().info('    Inclina ADELANTE → Forward')
        self.get_logger().info('    Inclina ATRÁS → Backward')
        self.get_logger().info('    Gira IZQUIERDA → Left')
        self.get_logger().info('    Gira DERECHA → Right')
    
    def leer_bmi_y_controlar(self):
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            # Leer datos del BMI160
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Parsear: X:valor,Y:valor
                if line.startswith('X:'):
                    parts = line.split(',')
                    x_val = int(parts[0].split(':')[1])  # -100 a 100
                    y_val = int(parts[1].split(':')[1])  # -100 a 100
                    
                    # Determinar comando según valores del joystick
                    comando = 'S'  # Stop por defecto
                    
                    # Umbral para activar movimiento (ajusta si es necesario)
                    umbral = 30
                    
                    if y_val > umbral:           # Adelante
                        comando = 'F'
                    elif y_val < -umbral:        # Atrás
                        comando = 'B'
                    elif x_val < -umbral:        # Izquierda
                        comando = 'L'
                    elif x_val > umbral:         # Derecha
                        comando = 'R'
                    
                    # Enviar solo si cambió el comando
                    if comando != self.ultimo_comando:
                        self.enviar_comando_ivan(comando, x_val, y_val)
                        self.ultimo_comando = comando
                    
        except Exception as e:
            self.get_logger().error(f'Error leyendo BMI: {e}')
    
    def enviar_comando_ivan(self, cmd, x, y):
        # Mostrar dirección con emojis
        emoji = {
            'F': '  ADELANTE',
            'B': '  ATRÁS',
            'L': '  IZQUIERDA',
            'R': '  DERECHA',
            'S': ' STOP'
        }
        
        if self.sock:
            try:
                self.sock.send(f'{cmd}\n'.encode())
                self.get_logger().info(
                    f' BMI[X:{x:4d} Y:{y:4d}]  IVAN: {emoji.get(cmd, cmd)}'
                )
            except Exception as e:
                self.get_logger().error(f'Error WiFi: {e}')
                # Intentar reconectar
                try:
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.connect((self.esp32_ip, self.esp32_port))
                    self.sock.settimeout(1)
                    self.get_logger().info(' WiFi reconectado')
                except:
                    self.sock = None
                    self.get_logger().error(' No se pudo reconectar WiFi')
        else:
            self.get_logger().info(f'[SIM] {emoji.get(cmd, cmd)} (X:{x} Y:{y})')
    
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Arduino desconectado')
        if self.sock:
            self.sock.close()
            self.get_logger().info('WiFi desconectado')
        super().destroy_node()

def main():
    rclpy.init()
    node = BMIIvanControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' Apagando sistema...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
