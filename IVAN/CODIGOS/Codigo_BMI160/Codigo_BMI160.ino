
#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=================================");
  Serial.println("  BMI160 - TEST DE DIAGNOSTICO");
  Serial.println("=================================");
  Serial.println();
  
  // Reset del sensor
  if (bmi160.softReset() != BMI160_OK){
    Serial.println(" ERROR: softReset falló");
    while(1);
  } else {
    Serial.println(" softReset OK");
  }
  
  delay(100);
  
  // Inicializar I2C
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println(" ERROR: I2cInit falló");
    Serial.println("   Revisa las conexiones I2C:");
    Serial.println("   SDA → A4");
    Serial.println("   SCL → A5");
    Serial.println("   VCC → 3.3V o 5V");
    Serial.println("   GND → GND");
    while(1);
  } else {
    Serial.println(" I2cInit OK");
  }
  
  Serial.println();
  Serial.println("Mueve el sensor para ver cambios");
  Serial.println("Si los valores NO cambian = problema de hardware");
  Serial.println();
  Serial.println("GyroX\tGyroY\tGyroZ\tAccX\tAccY\tAccZ");
}

void loop() {
  int16_t accelGyro[6] = {0};
  
  // Leer datos del sensor
  int rslt = bmi160.getAccelGyroData(accelGyro);
  
  if (rslt == 0) {
    // IMPRIMIR VALORES CRUDOS (sin filtros)
    // Esto debe cambiar cuando muevas el sensor
    
    // Giroscopio (velocidades angulares)
    Serial.print(accelGyro[0]); Serial.print("\t");
    Serial.print(accelGyro[1]); Serial.print("\t");
    Serial.print(accelGyro[2]); Serial.print("\t");
    
    // Acelerómetro
    Serial.print(accelGyro[3]); Serial.print("\t");
    Serial.print(accelGyro[4]); Serial.print("\t");
    Serial.println(accelGyro[5]);
    
  } else {
    Serial.println(" ERROR: getAccelGyroData falló");
  }
  
  delay(100);  // 10Hz para diagnóstico
}
