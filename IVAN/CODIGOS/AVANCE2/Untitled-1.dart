#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "IVAN_ROBOT";
const char* password = "12345678";

WiFiServer server(8888);
WiFiClient client;

const int AIN1 = 25;
const int AIN2 = 26;
const int PWMA = 27;
const int BIN1 = 32;
const int BIN2 = 33;
const int PWMB = 14;
const int STBY = 12;
const int velocidad = 255;

String comando = "";

void setup() {
  Serial.begin(115200);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  detener();
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  
  Serial.print("IVAN WiFi listo! Conéctate a: ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(IP);
  
  server.begin();
  Serial.println(" Esperando conexión...");
}

void loop() {
  // Manejar conexiones WiFi
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println(" Cliente conectado!");
    }
  }
  
  if (client && client.connected()) {
    if (client.available()) {
      comando = client.readStringUntil('\n');
      comando.trim();
      comando.toUpperCase();
      
      Serial.print(" Comando WiFi: ");
      Serial.println(comando);
      ejecutarMovimiento(comando);
    }
  }
  
  // También recibir por Serial (por si acaso)
  if (Serial.available()) {
    comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toUpperCase();
    ejecutarMovimiento(comando);
  }
}

void ejecutarMovimiento(String cmd) {
  if (cmd == "F") adelante();
  else if (cmd == "B") atras();
  else if (cmd == "L") izquierda();
  else if (cmd == "R") derecha();
  else if (cmd == "S") detener();
}

void adelante() {
  Serial.println("-> Adelante");
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, velocidad);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, velocidad);
}

void atras() {
  Serial.println("-> Atrás");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, velocidad);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, velocidad);
}

void izquierda() {
  Serial.println("-> Izquierda");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, velocidad);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, velocidad);
}

void derecha() {
  Serial.println("-> Derecha");
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, velocidad);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, velocidad);
}

void detener() {
  Serial.println("-> Stop");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}