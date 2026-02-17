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


volatile long leftTicks = 0;
volatile long rightTicks = 0;
bool volatile iss4Flaga = false; 
float v_ref_fuzzy = 0;           // Prędkość wyznaczona przez Fuzzy
unsigned long lastFuzzyTime = 0;

// PID dla kół (Niezależne)
struct WheelPID {
  float Kp_w = 4.5;
  float Ki_w = 1.2;
  float lastErr = 0;
  float accum = 0;
  long lastTicks = 0;
};
WheelPID pidL, pidR;


void countL() { leftTicks++; }
void countR() { rightTicks++; }

float get_wall_dist() {
  digitalWrite(TRIGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGER_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 400.0;
  return dur * 0.034 / 2.0;
}


float fuzzyCompute(float d) {
  float za_blisko = 0, blisko = 0, srednio = 0, daleko = 0;

  if (d <= 20) za_blisko = 1.0;
  else if (d < 25) za_blisko = (25 - d) / 5.0;

  if (d > 20 && d <= 30) blisko = (d - 20) / 10.0;
  else if (d > 30 && d < 40) blisko = (40 - d) / 10.0;

  if (d > 35 && d <= 55) srednio = (d - 35) / 20.0;
  else if (d > 55 && d < 75) srednio = (75 - d) / 20.0;

  if (d >= 65) daleko = 1.0;
  else if (d > 55 && d < 65) daleko = (d - 55) / 10.0;

  float v_stop = 0, v_bardzo_wolno = 35, v_wolno = 70, v_szybko = 140;

  float num = (za_blisko * v_stop) + (blisko * v_bardzo_wolno) + (srednio * v_wolno) + (daleko * v_szybko);
  float den = za_blisko + blisko + srednio + daleko;

  if (den == 0) return 0;
  return num / den;
}


int computeWheelPID(WheelPID &p, long currentTicks, float targetV) {
  float measuredV = currentTicks - p.lastTicks;
  p.lastTicks = currentTicks;
  float err = targetV - measuredV;
  p.accum = constrain(p.accum + err, -50, 50);
  float out = (p.Kp_w * err) + (p.Ki_w * p.accum);
  return constrain((int)out, 0, 255);
}


void PID(){
  
  static float integral_bal = 0; 
  float proportional = distance - distance_point;
  integral_bal = integral_bal + proportional * 0.1;
  derivative = (proportional - previousError) / 0.1;
  float output = kp * proportional + ki * integral_bal + kd * derivative;
  
  Serial.print(distance); Serial.print(" : "); Serial.println(output);
  previousError = proportional;
  myservo.write(constrain(servo_zero + output, 0, 180));
}

void PIZD(){
  if (millis() > myTime + t){
    distance = get_wall_dist(); 
    myTime = millis();
    PID();
  }
}

void ZaliczamSh() {
  Serial.print("Poloz pilke");
  delay(3000);
  float start = millis();
  float sum = 0; float count = 0;
  while (true) {
    float teraz = millis();
    if (teraz - start > 10000 && teraz - start <= 13000) {
      sum += abs(distance - distance_point); count++;
    } else if (teraz - start >= 13000) break;
    PIZD();
  }
  Serial.print("moje MOE: "); Serial.println(sum / count);
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
  integral += (long)error * (long)updTime;
  if (integral > 10000) integral = 10000;
  if (integral < -10000) integral = -10000;
  derivative = (float)(error - lastError) / (float)updTime;
  D_filter = D_alpha * derivative + (1.0 - D_alpha) * D_filter;
  int u = (int)(Kp * error + Ki * integral + Kd * D_filter);
  setMotorSpeed(SPEED - u, SPEED + u);
  lastError = error;
}

int ControlSUM(String data) {
  int sum = 0;
  for (unsigned int i = 0; i < data.length(); i++) sum += data[i];
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
  else if (command == "IR") {
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
  else if (command == "ISS4") {
    iss4Flaga = true; jazdaFlaga = false;
    digitalWrite(LEFT_MOTOR_FWD, HIGH); digitalWrite(LEFT_MOTOR_REV, LOW);
    digitalWrite(RIGHT_MOTOR_FWD, HIGH); digitalWrite(RIGHT_MOTOR_REV, LOW);
    Serial.println("START ISS4 - JAZDA DO SCIANY");
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

// void PID(){
//   float proportional = distance-distance_point;
//   integral = integral+proportional*0.1;
//   derivative=(proportional-previousError)/0.1;
//   float output=kp*proportional+ki*integral+kd*derivative;
//   Serial.print(distance);
//   Serial.print(" : ");
//   Serial.print(proportional);
//   Serial.print(" : ");
//   Serial.println(output);
//   previousError=proportional;
//   myservo.write(servo_zero+output);
// }

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


  pinMode(TRIGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myservo.attach(9);   // zmień 9 jeśli serwo jest na innym pinie

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), countL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), countR, RISING);




}



void loop() {

  if(testFlaga){
    PIZD();
  }
  
  // while (jazdaFlaga) {

  //   unsigned int now = millis();

  //   if (t - lastPID >= updTime) {
  //     if (kalibracjaOK) updatePID();
  //     lastPID = t;
  //   }

  //   if (Serial.available()) {
  //     String incomingData = Serial.readStringUntil('\n');
  //     unpackCommand(incomingData);
  //   }
  // }


  if (iss4Flaga) {
    unsigned long now = millis();
    Serial.println("JADE");
    if (now - lastFuzzyTime >= 50) {
      lastFuzzyTime = now;
      float d_curr = get_wall_dist();

      if (d_curr < 10.0) {
        setMotorSpeed(0, 0);
        iss4Flaga = false;
        Serial.println("EMERGENCY STOP < 10cm");
      } 
      else if (d_curr <= 20.5 && v_ref_fuzzy < 5) {
        setMotorSpeed(0, 0);
        iss4Flaga = false;
        Serial.print("OSIAGNIETO 20cm. Dystans: "); Serial.println(d_curr);
      } 
      else {
        v_ref_fuzzy = fuzzyCompute(d_curr);
      }
    }

    static unsigned long lastWheelPIDTime = 0;
    if (now - lastWheelPIDTime >= 20) {
      lastWheelPIDTime = now;
      float target_t = v_ref_fuzzy / 10.0; 
      int pwmL = computeWheelPID(pidL, leftTicks, target_t);
      int pwmR = computeWheelPID(pidR, rightTicks, target_t);
      analogWrite(LEFT_MOTOR_PWM, pwmL);
      analogWrite(RIGHT_MOTOR_PWM, pwmR);
    }
  }




  if (Serial.available()) {
    
    String incomingData = Serial.readStringUntil('\n');
    unpackCommand(incomingData);
  }

  delay(1); 
}
