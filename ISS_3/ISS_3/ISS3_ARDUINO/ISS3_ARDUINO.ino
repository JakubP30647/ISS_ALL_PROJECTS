#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <TRSensors.h>

#define TRIGER_PIN 11
#define ECHO_PIN 12
#define ANALOG_READ_IR_LEFT A4
#define DIGITAL_READ_IR_LEFT 7
#define ANALOG_READ_IR_RIGHT A5
#define DIGITAL_READ_IR_RIGHT 8
 
#define PIN_LEFT_MOTOR_SPEED 5
#define PIN_LEFT_MOTOR_FORWARD A1     // zmiana A1 na A0   LEWE     
#define PIN_LEFT_MOTOR_REVERSE A0
#define PIN_LEFT_ENCODER 2
   
#define PIN_RIGHT_MOTOR_SPEED 6
#define PIN_RIGHT_MOTOR_FORWARD A2   // zmiana A2 na A3 PRAWE
#define PIN_RIGHT_MOTOR_REVERSE A3
#define PIN_RIGHT_ENCODER 3


#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_FWD A0
#define LEFT_MOTOR_REV A1
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_FWD A2
#define RIGHT_MOTOR_REV A3
#define NUM_SENSORS 5
#define SERIAL_BAUD_RATE 9600

TRSensors trs;
unsigned int sensorValues[NUM_SENSORS];


unsigned long myTime;
Servo myservo;
float volatile distance;
float volatile kp = 2.7;
float volatile ki = 0;
float volatile kd = 1.3;
//float integral = 0.0;
//float derivative = 0.0;
float previousError = 0.0;
float volatile distance_point = 23;
int volatile servo_zero = 95;
int t = 100;
bool volatile testFlaga = false;



float Kp = 0.06;
float Ki = 0.0;
float Kd = 2.8;
long integral = 0;
int lastError = 0;
float derivative = 0;
float D_filter = 0;
float D_alpha = 0.2; // filtr dolnoprzepustowy D
int PWM_MAX = 255;
int PWM_MIN = 0;
int SPEED = 85;
int updTime = 5; // ms

long lastPID = 0;





//unsigned long t = 0;

bool volatile jazdaFlaga = false;      // JAZDA START dopiero po LFJAZDA
bool volatile kalibracjaOK = false;

// bool volatile stoppdfill = false;

void PIZD(){
    if (millis() > myTime+t){
    distance = get_dist(100); 
    myTime = millis();
    PID();
  }
}

void ZaliczamSh() {
  Serial.print("Poloz pilke");
  delay(3000);

  float start = millis();
  float sum = 0;
  float count = 0;

  while (true) {
    float teraz = millis();

    if (teraz - start > 10000 && teraz - start <= 13000) {
      sum += abs(distance - distance_point);
      count++;
    } else if (teraz - start >= 13000) {
      break;
    }

    PIZD();
  }

  float MOE = sum / count;

  Serial.print("moje MOE: ");
  Serial.println(MOE);
  //Serial.println("PS. prosze oby 20pkt");
}



void setMotorSpeed(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, PWM_MIN, PWM_MAX);
  rightPWM = constrain(rightPWM, PWM_MIN, PWM_MAX);
 
  analogWrite(LEFT_MOTOR_PWM, leftPWM);
  analogWrite(RIGHT_MOTOR_PWM, rightPWM);
}



void updatePID() {
  unsigned int pos = trs.readLine(sensorValues); 
  int error = (int)pos - 2000;
 
  
  integral += error * updTime;
  if (integral > 10000) integral = 10000;
  if (integral < -10000) integral = -10000;
 
  derivative = (error - lastError) / updTime;
  
  D_filter = D_alpha * derivative + (1 - D_alpha) * D_filter;
 
  int u = (int)(Kp * error + Ki * integral + Kd * D_filter);
 
  int leftPWM = SPEED - u;
  int rightPWM = SPEED + u;
 
  setMotorSpeed(leftPWM, rightPWM);
 
  lastError = error;
 
}




int ControlSUM(String data) {
  int sum = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    sum += data[i];
  }
  return sum % 256;  
}

void unpackCommand(String frame) {
  boolean key = false;
  
  if (frame[0] != '\x02' || frame[frame.length() - 1] != '\x03') {
    Serial.println("Błąd: Niepoprawna ramka (brak znaku początkowego lub końcowego)");
    return;
  }

  
  frame = frame.substring(1, frame.length() - 1); 

 
  int colonIndex = frame.indexOf(':');
  String command;
  String param;

  int firstColon = frame.indexOf(':');
  int secondColon = frame.indexOf(':', firstColon + 1);

  if (firstColon == -1) {
  Serial.println("Błąd: brak pierwszego ':'");
  return;
  }

  String message = frame.substring(0, firstColon);
  String argument1;
  String argument2 = "";

  if (secondColon != -1) {
    argument1 = frame.substring(firstColon + 1, secondColon);
    argument2 = frame.substring(secondColon + 1);
  } else {
    argument1 = frame.substring(firstColon + 1);
  }

  int checksumReceived = 0;
  int checksumCalculated = 0;

  if (argument2.length() > 0) {
    checksumCalculated = ControlSUM(message + ":" + argument1);
    checksumReceived = argument2.toInt();
    
  } else {
    checksumCalculated = ControlSUM(message);
    checksumReceived = argument1.toInt();
    
  }
  


  if (colonIndex != -1) {
   
    command = frame.substring(0, colonIndex);
    param = frame.substring(colonIndex + 1, secondColon);
    key=true;
  } else {
    
    command = frame;
    param = "";
    key=true;
  }

 
  
  

  if (checksumReceived == checksumCalculated && key==true) {
    
    key=false;
    Serial.println("ACK");
    Serial.print("Otrzymano komendę: ");
    Serial.println(command + " ");
    if (param != "" && argument2 != "") {
      Serial.print("Parametr: ");
      Serial.println(param + " ");
    }

  if (command == "V") {
    Serial.println("Zmiana predkosci poruszania na " + param);
  } 
  else if (command == "STATUS") {
    Serial.println("Wyswietlanie statusu robota.");
  }
  else if(command == "STOPTEST"){
    testFlaga = false;
    Serial.println("Zastopowałem test pozdro.");
  }
  else if (command == "PRZOD") {
    Serial.println("Jazda do przodu o " + param );
  } 
  else if (command == "TYL") {
    Serial.println("Jazda do tylu o " + param);
  } 
  else if (command == "LEWO") {
    Serial.println("Skret w lewo o "  + param);
  } 
  else if (command == "PRAWO") {
    Serial.println("Skret w prawo o " + param );
  } 
  else if (command == "STOP") {
    Serial.println("Natychmiastowe zatrzymanie.");
  } 
  else if (command == "B") {
    Serial.println("Odczyt sonaru");
  } 
  else if (command == "I") {
    Serial.println("Odczyt czujnika IR");
  }
  else if (command == "PING") {
    
    Serial.println("PONG");
  }   else if (command == "P") {
    kp = param.toInt();
    Serial.println(String("Ustawianie wartości P na ") + kp);
  }
  else if (command == "I") {
    ki = param.toInt();
    Serial.println("Ustawianie wartości I na " + param);
  }
  else if (command == "D") {
    kd = param.toInt();
    Serial.println("Ustawianie wartości D na " + param);
  }
  else if (command == "ZERO") {
    servo_zero = param.toInt();
    Serial.println("Ustawianie pozycji ZERO serwa na " + param);
  }
  else if (command == "DIST") {
    distance = param.toInt();
    Serial.println("Ustawianie docelowej odległości (TARGET) na " + param);
  }
  else if (command == "TEST") {
    testFlaga = true;
    Serial.println("Tryb testowy PIZD (telemetria)");
    
  }
  else if (command == "RUN") {
    ZaliczamSh();
    Serial.println("Tryb zaliczeniowy (10s + MAE)");
  }
  if (command == "LFJAZDA") {

    Serial.println("Rozpoczynam LF");

    if (!kalibracjaOK) {
      Serial.println("Kalibracja TRSensors...");
      for (int i = 0; i < 400; i++) trs.calibrate();
      Serial.println("Kalibracja OK.");
      kalibracjaOK = true;
    }

    
    jazdaFlaga = true;

    analogWrite(LEFT_MOTOR_PWM, SPEED);
    analogWrite(RIGHT_MOTOR_PWM, SPEED);

    Serial.println("WYSTARTOWANO line-follow");
  }
  else {
    Serial.println("Nieznana komenda.");
  }

  
    
   Serial.println("ENDENDEND");
    
  } else {
    
    key=false;
    Serial.print("Blad: Niepoprawna suma kontrolna ");
    Serial.print(checksumReceived);
    Serial.print( "  ");
    Serial.print(checksumCalculated);
    Serial.println("  " + frame);
    

  }

}
// P2


float get_dist(int n){
  long sum=0;
  for(int i=0;i<n;i++){
    sum=sum+analogRead(A0);
  }  
  float adc=sum/n;

  float distance_cm = 17569.7 * pow(adc, -1.2062);
  return(distance_cm);
}

void PID(){
  float proportional = distance-distance_point;
  integral = integral+proportional*0.1;
  derivative=(proportional-previousError)/0.1;
  float output=kp*proportional+ki*integral+kd*derivative;
  Serial.print(distance);
  Serial.print(" : ");
  Serial.print(proportional);
  Serial.print(" : ");
  Serial.println(output);
  previousError=proportional;
  myservo.write(servo_zero+output);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_REV, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_REV, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Wymuszone stanie silników na starcie
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_REV, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_REV, LOW);

  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);

  Serial.println("Robot gotowy. Czekam na LFJAZDA...");
}



void loop() {

  if(testFlaga){
    PIZD();
  }
  
  while (jazdaFlaga) {

    unsigned int t = millis();

    if (t - lastPID >= updTime) {
      if (kalibracjaOK) updatePID();
      lastPID = t;
    }

    if (Serial.available()) {
      String incomingData = Serial.readStringUntil('\n');
      unpackCommand(incomingData);
    }
  }


  if (Serial.available()) {
    
    String incomingData = Serial.readStringUntil('\n');
    unpackCommand(incomingData);
  }

  delay(1); 
}
