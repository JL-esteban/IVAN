#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;

// Variables para valores tipo joystick
int joystickX = 0;  // Izquierda/Derecha (-100 a 100)
int joystickY = 0;  // Adelante/Atrás (-100 a 100)

// Zona muerta para evitar drift
const int DEADZONE = 15;

// Umbrales para mapear giroscopio a valores de joystick
const int GYRO_MIN = 100;   // Movimiento mínimo para empezar a contar
const int GYRO_MAX = 1000;  // Movimiento máximo (se mapea a 100%)

// Filtro de suavizado
const float ALPHA = 0.6;
float filteredX = 0;
float filteredY = 0;

// Decay para volver a centro cuando no hay movimiento
const float DECAY_RATE = 0.85;  // Qué tan rápido vuelve a centro

void setup(){
  Serial.begin(115200);
  
  // Intentar inicializar
  if (bmi160.I2cInit(0x69) != BMI160_OK && bmi160.I2cInit(0x68) != BMI160_OK){
    while(1);  // Si falla, detener
  }
  
  delay(1000);
}

void loop(){
  int rslt;
  int16_t accelGyro[6]={0}; 
  
  // Leer datos del sensor
  rslt = bmi160.getAccelGyroData(accelGyro);
  
  if(rslt == 0){
    // Giroscopio
    int16_t gx = accelGyro[3];  // Roll (izquierda/derecha)
    int16_t gy = accelGyro[4];  // Pitch (adelante/atrás)
    
    // Mapear valores de giroscopio a rango -100 a 100
    int rawX = 0;
    int rawY = 0;
    
    // Eje X (izquierda/derecha)
    if (abs(gx) > GYRO_MIN) {
      rawX = map(constrain(gx, -GYRO_MAX, GYRO_MAX), -GYRO_MAX, GYRO_MAX, -100, 100);
    }
    
    // Eje Y (adelante/atrás) - invertido porque gy negativo es adelante
    if (abs(gy) > GYRO_MIN) {
      rawY = map(constrain(gy, -GYRO_MAX, GYRO_MAX), -GYRO_MAX, GYRO_MAX, -100, 100);
      rawY = -rawY;  // Invertir para que adelante sea positivo
    }
    
    // Aplicar filtro de suavizado
    filteredX = ALPHA * filteredX + (1 - ALPHA) * rawX;
    filteredY = ALPHA * filteredY + (1 - ALPHA) * rawY;
    
    // Aplicar decay (volver a centro gradualmente)
    if (abs(gx) < GYRO_MIN) {
      filteredX *= DECAY_RATE;
    }
    if (abs(gy) < GYRO_MIN) {
      filteredY *= DECAY_RATE;
    }
    
    // Convertir a enteros
    joystickX = (int)filteredX;
    joystickY = (int)filteredY;
    
    // Aplicar zona muerta
    if (abs(joystickX) < DEADZONE) joystickX = 0;
    if (abs(joystickY) < DEADZONE) joystickY = 0;
    
    // Limitar rango
    joystickX = constrain(joystickX, -100, 100);
    joystickY = constrain(joystickY, -100, 100);
    
    // Enviar datos en formato para ROS
    // Formato: X:valor,Y:valor
    Serial.print("X:");
    Serial.print(joystickX);
    Serial.print(",Y:");
    Serial.println(joystickY);
  }
  
  delay(50);  // 20Hz
}