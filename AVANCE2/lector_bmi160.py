#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import re

class BMI160Reader(Node):
    def __init__(self):
        super().__init__('bmi160_reader')
        
        # Parámetros configurables
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 20.0)
        
        # Obtener parámetros
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Publisher para los comandos de joystick
        self.publisher = self.create_publisher(Twist, 'bmi_joystick', 10)
        
        # Inicializar puerto serial
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Conectado a {port} a {baud} baudios')
            # Limpiar buffer inicial
            self.serial_port.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir puerto serial: {e}')
            raise
        
        # Timer para leer datos
        timer_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
        self.get_logger().info('Nodo BMI160 Reader iniciado correctamente')
    
    def read_and_publish(self):
        """Lee datos del serial y los publica como mensaje Twist"""
        try:
            if self.serial_port.in_waiting > 0:
                # Leer línea del serial
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Parsear datos: formato "X:valor,Y:valor"
                match = re.match(r'X:(-?\d+),Y:(-?\d+)', line)
                
                if match:
                    x_raw = int(match.group(1))
                    y_raw = int(match.group(2))
                    
                    # Crear mensaje Twist
                    msg = Twist()
                    
                    # Mapear valores (-100 a 100) a velocidades normalizadas (-1.0 a 1.0)
                    msg.linear.x = y_raw / 100.0   # Adelante/Atrás
                    msg.angular.z = -x_raw / 100.0  # Izquierda/Derecha (negativo para que sea intuitivo)
                    
                    # Los demás valores en 0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    
                    # Publicar
                    self.publisher.publish(msg)
                    
                    # Log solo cuando hay movimiento
                    if abs(x_raw) > 0 or abs(y_raw) > 0:
                        self.get_logger().debug(
                            f'Joystick - X: {x_raw:4d} Y: {y_raw:4d} | '
                            f'Twist - linear.x: {msg.linear.x:5.2f} angular.z: {msg.angular.z:5.2f}'
                        )
                else:
                    self.get_logger().warning(f'Formato inválido: {line}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f'Error de serial: {e}')
        except Exception as e:
            self.get_logger().error(f'Error inesperado: {e}')
    
    def destroy_node(self):
        """Limpieza al cerrar el nodo"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Puerto serial cerrado')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = BMI160Reader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()