#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class BMIJoystickNode(Node):
    def __init__(self):
        super().__init__('bmi_joystick_node')
        
        self.get_logger().info(' BMI160 Joystick Virtual - Iniciando...')
        
        # Publicador en el topic /joy (igual que el control PS4)
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        
        # Conectar al Arduino
        puerto_serial = '/dev/ttyACM0'  # Cambia si es necesario
        
        try:
            self.ser = serial.Serial(puerto_serial, 115200, timeout=1)
            time.sleep(2)  # Esperar inicialización del Arduino
            self.get_logger().info(f' Conectado a Arduino en {puerto_serial}')
        except Exception as e:
            self.get_logger().error(f' Error conectando Arduino: {e}')
            self.get_logger().error('Puertos comunes: /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyACM1')
            self.ser = None
            return
        
        # Timer para leer datos del Arduino (20Hz)
        self.timer = self.create_timer(0.05, self.leer_bmi)
        
        self.get_logger().info('BMI160 listo! Publicando en /joy')
    
    def leer_bmi(self):
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            # Leer si hay datos disponibles
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Parsear formato: X:valor,Y:valor
                if line.startswith('X:'):
                    parts = line.split(',')
                    x_str = parts[0].split(':')[1]
                    y_str = parts[1].split(':')[1]
                    
                    x_val = int(x_str)
                    y_val = int(y_str)
                    
                    # Crear mensaje Joy
                    joy_msg = Joy()
                    joy_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Normalizar a rango -1.0 a 1.0
                    # axes[0] = X (izquierda/derecha)
                    # axes[1] = Y (adelante/atrás)
                    joy_msg.axes = [
                        float(x_val) / 100.0,
                        float(y_val) / 100.0
                    ]
                    joy_msg.buttons = []
                    
                    # Publicar en /joy
                    self.publisher.publish(joy_msg)
                    
                    # Log solo cuando hay movimiento
                    if abs(x_val) > 5 or abs(y_val) > 5:
                        self.get_logger().info(
                            f'🎮 BMI → /joy [X:{x_val:4d} Y:{y_val:4d}]',
                            throttle_duration_sec=0.5
                        )
                    
        except Exception as e:
            self.get_logger().error(f'Error leyendo BMI: {e}')
    
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Arduino desconectado')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BMIJoystickNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' Cerrando BMI Joystick...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
