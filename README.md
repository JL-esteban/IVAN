En este github mostramos las conexiones, codigos tanto en arduino como en ros2, tambien simulación en RViz, que nos llevo a fformular este proyecto?, a quien no le gustaria controlar algo con solo el mover de sus manos?, que sea algo entretenido a la hora de sus terapias de recuperacion?, que las personas lo puedan usar tanto a la hora de su terapia como un juguete.

Así que comenzamos, tenemos de la siguiente fora su conexión y la cantidad de objetos,componentes,baterias y microchips que usamos.

Vamos a comenzar con el principal, la mano, la mano es aquella que va a ser de control con los movimientos de su muñeca, tu te preguntaras, como?, muy sencillo hay un modulo que se llama BMI160 el cual es un giroscopio y acelerometro, este modulo se programo para que soltara las coordenadas X Y, asi convertimos con el acelerometro que mande las posiciones como si fuera un mando,  no sabes que hace un mando?, te lo explico:

El mando tiene una serie de nombres los cuales las computadoras detectan como por ejemplo cada palanca, movimiento o boton por un axis, en nuestro caso es un R3 de un mando (es la palanca de la izquierda) esta palanca tiene dos axis los cuales son axis0 y axis1, el axis0 son los movimientos de izquierda y derecha, el axis1 son los movimientos de arriba y abajo, estos sueltan unos numeros los cuales el computador traduce para generar un movimiento a un personaje.

Teniendo eso en mente se supo que con las coordenadas de la mano y su giro podiamos hacer un joystick, el que lee eso es un Arduino UNO, el cual obtiene los datos y se los manda a el computador.

Ahora el computador con los datos del BMI160 ya como un joystick mandara la señal atravez de wifi al microchip del semisumergible que esta en el agua para poder moverse.

Ahora te explicare que tiene el semisumergible, que los autores de este repositorio lo llamaron como IVAN,IVAN tiene un ESP32, el cual resive los datos del computador atravez del wifi que son las direcciones dadas por el joystick(BMI160), este ESP32 le mandara la señal a un puente h, el cual es un tb6612fng, y este segun las direcciones dadas comenzara a mandar el voltaje a las salidas de A0 y A1 que son las salidas del primer motor y B0 B1 que son las salidas del segundo motor, todos los gnds de esta mismas estan conectadas entre ellas, y asi este se desplazaria por el agua.

No nos olvidemos de lo mas importante ¿conque alimentamos a IVAN?, IVAN tiene dos tipos de alimentación interna, una que es solo para el ESP32 y el otro el cual es solo para los motores, para alimentar el ESP32 se utilizaron 3 paquetes de pilas AA y esta salida de voltaje que dan 9V entran a un regulador de voltaje creado (que aqui se pasara la kicad) para que asi entren 9V y salgan 5V un buen voltaje con un buen amperaje que salen de las pilas para mantener prendido el ESP32 por un largo rato aproximadamente 30-40min, el tb6612fng  tiene dos pines uno llamado VCC y otro VM el VCC es aquel que resive entre 3.3V a 5V, los autores del proyecto alimentaron a 5V el tb6612fng y el VM a una bateria de 7.4V 1500mA


-------------------------------------------------------------------------------------------------------------------------------------------
EL CODIGO, LO IMPORTANTE QUE NECESITAS SABER
-------------------------------------------------------------------------------------------------------------------------------------------

  el que se llama Codigo_BMI160 es aquel codigo que conectaras el BMI160 con el ARDUINO UNO, a continuaciòn te explico como conectarlo:

  El BMI160 de la placa que yo te paso es realmente de 4 pines que es VCC,GND,SDA,SCL
, esos se pondran de la siguiente forma con el ARDUINO UNO, y es:
ESP32             |ARDUINO UNO
VCC --------------->5V
GND---------------->GND
SCL----------------->A5
SDA----------------->A4



con esas conexiones y el codigo de arduino que se otorgo te debe comenzar a mostrar coordenadas en la terminal.

Luego las conexiones de el ESP32 son de la siguiente forma:

Tb6612fng           |  ESP32
AIN1 ----------------> 25;
AIN2 ----------------> 26;
PWMA ----------------> 27;
BIN1 ----------------> 32;
BIN2 ----------------> 33;
PWMB ----------------> 14;
STBY ----------------> 12;



lo que es VM y VCC fue explicado anteriormente, y los puntos A0.A1.B0.B1 son los puntos de los motores A y B, que tipo de motores usamos?, usamos motores Hélice submarina cepillada BM70.



-----------------------------------------------------------------------------------------------------------------------------------------
una vez que has tenido esto vamos ha abrir el RVizte 
#iremos a la carpeta de ros2_ws
cd ~/ros2_ws




#cada vez que hagas un cambio o abras por primera vez una terminal en ubuntu debes poner el siguiente com  ando es ley  
colcon build --packages-select robot_boat

#cargamos el entorno
source install/setup.bash

#y ejecuta para verlo en rviz
ros2 launch robot_boat display.launch.py 



------------------------------------------------------------------------------------------------------------------------------------------
Ahora para cargar los codigos hay que hacerse de la siguiente manera
Vas al archivo de "AVANCE2" que esta en el principio del GitHub y abres dos terminales y ejecutas los siguientes comandos
# Terminal 1 - Lector BMI
source /opt/ros/humble/setup.bash
python3 lector_bmi160.py --ros-args -p serial_port:=/dev/ttyUSB0

# Terminal 2 - Control barco (después de que Terminal 1 funcione)
source /opt/ros/humble/setup.bash
python3 bmi_to_boat_control.py --ros-args -p esp32_ip:=192.168.4.1

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/9bf23261-350b-44c3-a943-137fd867ff90" />

<img width="1600" height="900" alt="image" src="https://github.com/user-attachments/assets/8ee88e2e-455d-4cba-9380-97dc207197f7" />



