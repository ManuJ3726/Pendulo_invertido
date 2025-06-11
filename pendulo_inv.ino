#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

//Pines
const int pinPWMA=5;
const int pinAIN2=18;
const int pinAIN1=0;
const int pinPWMB=17;
const int pinBIN1=4;
const int pinBIN2=16;
const int pinSTBY=15;

//Variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x = 0.0, ang_y = 0.0;
float ang_x_prev = 0.0, ang_y_prev = 0.0;

//PID
float Kp = 15.0;
float Ki = 1.1;
float Kd = 2.0;
float integral = 0.0;
float derivada = 0.0;
float previous_error = 0.0;

void setup() {
  Serial.begin(57600);
  Wire.begin(21,22);
  sensor.initialize();

  //defino los pines
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

}

void loop() {
  //obtengo aceleración y rotacion
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  float aceleracion_ang_x=atan2(ay, sqrt(ax * ax + az * az)) * (180.0/PI);

  dt = (millis() - tiempo_prev)/ 1000.0;
  tiempo_prev = millis();

  //saco el angulo x actual y lo resto con 0 para obtener el error
  ang_x=0.98 * (ang_x_prev + (gx/131.0) * dt) + 0.02 * aceleracion_ang_x;
  ang_x_prev = ang_x;

  float angulo_buscado = -5.0;

  float error = angulo_buscado - ang_x;

  integral += error * dt;
  derivada = (error - previous_error) / dt;

  //obtengo la velocidad
  float velocidad = (Kp * error + Ki * integral + Kd * derivada);
  previous_error = error;

  //enciendo motores
  enableMotors();
  //digitalWrite(pinBIN1, LOW);
  //digitalWrite(pinBIN2, HIGH);

  //digitalWrite(pinAIN1, LOW);
  //digitalWrite(pinAIN2, HIGH);

  //digitalWrite(pinSTBY, HIGH);
  //digitalWrite(pinPWMA, 100);
  //digitalWrite(pinPWMB, 100);
  //y les mando el valor que deben tomar y su velocidad
  moveMotor(pinPWMA, pinAIN2, pinAIN1, error);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, error);

  //muestro los datos actuales
  Serial.print("Ángulo X: "); Serial.print(ang_x);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Salida PID: "); Serial.print(velocidad); 
  Serial.print(" | Kp: "); Serial.print(Kp);
  Serial.print(" | Ki: "); Serial.print(Ki);
  Serial.print(" | Kd: "); Serial.println(Kd);

  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd){
      case 'q': Kp += 1; break;
      case 'a': Kp -= 1; break;
      case 'w': Ki += 0.1; break;
      case 's': Ki -= 0.1; break;
      case 'e': Kd += 0.1; break;
      case 'd': Kd -= 0.1; break;
    }
  Serial.print("Nuevo Kp: "); Serial.print(Kp);
  Serial.print(" | Ki: "); Serial.print(Ki);
  Serial.print(" | Kd: "); Serial.println(Kd);
  }

  delay(5);

}

void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed){
  if (speed > 0.5){
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    digitalWrite(pinPWM, speed/3);
    } else
    
  if (speed < 0.5){
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    digitalWrite(pinPWM, -speed/3);
  }
}
void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}









