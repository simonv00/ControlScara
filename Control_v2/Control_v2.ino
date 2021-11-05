#include <Servo.h>
#include <util/atomic.h>
#include <math.h> 

//Creacion un objeto de tpo servo
Servo servoMotor;

//Definicion los pines digitales de los motores
#define Pin1_Motor1  22
#define Pin2_Motor1  23
#define Pin1_Motor2  28
#define Pin2_Motor2  29

//Definicion de los pines PWM de los motores
#define PWM_Motor1  8
#define PWM_Motor2  9

//Definicion de los pines que leen los encoders de los motores, tiene que ser de interrupciones
#define Encoder1_Motor1  2
#define Encoder2_Motor1  6
#define Encoder1_Motor2  3
#define Encoder2_Motor2  7

//Definicion del pin del servomotor
#define servo1  52
#define servo2  53
#define servoPWM  11
const int pwmVal = 155;

//Definicion de los pines de los finales de carrera
#define Pulsador1_Brazo1  38
//#define Pulsador2_Brazo1  39
#define Pulsador1_Brazo2  40
//#define Pulsador2_Brazo2  41

//Definicion del pin del actuador neumatico
#define Neumatica  44

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  

int accion = 0;
int ingresoX = 0;
int ingresoY = 0;

boolean newData = false;

bool inicio = false;

const float kp1 = 5;
const float kp2 = 3;
const float Kd = 0.025;
long prevT = 0;
float eprev1 = 0;
float eprev2 = 0;

const float q0_1 = 0.7124;
const float q1_1 = 0.4388;
const float q0_2 = 0.4591;
const float q1_2 = 0.4795;

const float Pi = 3.14159;

const int L1 = 100;
const int L2 = 165;

volatile int posi1 = 0; 
volatile int posi2 = 0; 
int contHome = 0;
int L = 0;

int X = 0;
int Y = 0;

float theta1 = 0;
float theta2 = 0;

float thetaRampa1 = 0;
float thetaRampa2 = 0;

const int maxSpeed = 150;
const int movServo = 90;

bool entrada = false;
bool neumatica = false;
bool rampa = false;
bool caja = false;
bool recogido = false;

const int rampaX = 100;
const int rampaY = -220;
int cajaX = 0;
int cajaY = 0;

float servoInicial = 0;
float neumaticaFinal = 0;

void setup(){
  Serial.begin(115200);
  //Encoders como entradas
  pinMode(Encoder1_Motor1, INPUT);
  pinMode(Encoder2_Motor1, INPUT);
  pinMode(Encoder1_Motor2, INPUT);
  pinMode(Encoder2_Motor2, INPUT);
  //interrupciones en los encoders
  attachInterrupt(digitalPinToInterrupt(Encoder1_Motor1),leerEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder1_Motor2),leerEncoder2,RISING);
  //Configura Motor
  pinMode(Pin1_Motor1, OUTPUT);
  pinMode(Pin2_Motor1, OUTPUT);
  pinMode(Pin1_Motor2, OUTPUT);
  pinMode(Pin2_Motor2, OUTPUT);
  //Pulsadores como entradas
  pinMode(Pulsador1_Brazo1, INPUT_PULLUP);
  pinMode(Pulsador1_Brazo2, INPUT_PULLUP);
  //Servo motor
  pinMode(servo1, OUTPUT);
  pinMode(servo2, OUTPUT);
  analogWrite(servoPWM, pwmVal);
  //Neumatica
  pinMode(Neumatica, OUTPUT);
  //PWM
  pinMode(PWM_Motor1, OUTPUT);
  pinMode(PWM_Motor2, OUTPUT);

  Serial.println("target pos");
}

void Recoger(float e1, float e2){
  //Serial.print("e1: ");
  //Serial.println(e1);
  //Serial.print("e2: ");
  //Serial.println(e2);
  //Serial.println("X: ");
  //Serial.println(X);
  //Serial.println("Y: ");
  //Serial.println(Y);
  if(X == rampaX && Y == rampaY && abs(e1) < 10 && abs(e2) < 10){
    rampa = true;
    caja = false;
    Serial.println("Rampa");
  }
  else if(X == cajaX && Y == cajaY && abs(e1) < 10 && abs(e2) < 10){
    caja = true;
    rampa = false;
  }
  NeumaticaAccion();
  ServoAccion();
  MovimientoSoltar();
}

void ServoAccion(){
  if(rampa && neumatica && entrada && !recogido && servoInicial < 10){
    servoInicial = micros();
    digitalWrite (servo1, LOW);
    digitalWrite (servo2, HIGH);
  }
  float currT = micros();
  float difT = ((float) (currT - servoInicial))/( 1.0e6 );
  
  if(difT >= 1 && difT <= 1.1 && servoInicial > 10){
    digitalWrite (servo1, LOW);
    digitalWrite (servo2, LOW);
  }
  else if(difT >= 2.5 && difT <= 2.6 && servoInicial > 10){
    digitalWrite (servo1, HIGH);
    digitalWrite (servo2, LOW);
  }
  else if(difT >= 3.6 && difT <= 3.7 && servoInicial > 10){
    digitalWrite (servo1, LOW);
    digitalWrite (servo2, LOW);
    recogido = true;
  } 
}

void NeumaticaAccion(){
  if(rampa && !recogido && !neumatica && entrada){
      Serial.println("Neumatica");
      neumatica = true;
      digitalWrite(Neumatica,HIGH);
  }
  else if (caja && recogido && neumatica && entrada && neumaticaFinal < 10){
      neumaticaFinal = micros();
  }
  float currT = micros();
  float difT = ((float) (currT - neumaticaFinal))/( 1.0e6 );
  if(difT > 4 && neumaticaFinal > 10){
    neumatica = false;
    
    entrada = false;
    recogido = false;
    digitalWrite(Neumatica,LOW);
  }
}

void MovimientoSoltar(){
  if(rampa && recogido){
    X = cajaX;
    Y = cajaY;
    inverseKinematics();
    rampa = false;
  }
}

void Take(int x, int y){
  servoInicial = 0;
  neumaticaFinal = 0;
  entrada = true;
  rampa = false;
  caja = false;
  setRampa();
  setCaja(x,y);
}

void setRampa(){
  X = rampaX;
  Y = rampaY;
  inverseKinematics();
}

void setCaja(int x, int y){
  cajaX = x;
  cajaY = y;
}

void loop(){   

  if(!inicio){
    Stop();
  }
  else{
    if(thetaRampa1 < theta1){
      thetaRampa1 += 0.5;
    }
    else if(thetaRampa1 > theta1){
      thetaRampa1 -= 0.5;
    }
    if(thetaRampa2 < theta2){
      thetaRampa2 += 0.5;
    }
    else if(thetaRampa2 > theta2){
      thetaRampa2 -= 0.5;
    }

    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    
    float thetaActual1 = 0;
    float thetaActual2 = 0;
    float ayuda  =0; 
    float ayuda2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      thetaActual1 = Pulsos_to_Grados1(posi1);
      ayuda = posi1;
      thetaActual2 = Pulsos_to_Grados2(posi2);
      ayuda2 = posi2;
    }
    // error
    float e1 = thetaActual1 - theta1;
    float e2 = thetaActual2 - theta2;
    
    // derivative
    float dedt1 = (e1-eprev1)/(deltaT);
    float dedt2 = (e2-eprev2)/(deltaT);

    // control signal
    //float u1 = kp1*e1 + Kd*dedt1;
    //float u2 = kp2*e2 + Kd*dedt2;
    float u1 = q0_1*e1 + q1_1*eprev1;
    float u2 = q0_2*e2 + q1_2*eprev2;
    
    eprev1 = e1;
    eprev2 = e2;
    
    float pwr1 = PWM1(u1);
    float pwr2 = PWM2(u2);
    int dir1 = setDir(u1);
    int dir2 = setDir(u2);
    
    plotInfo(thetaActual1, thetaActual2);
    
    setMotor(dir1,PWM_Motor1,pwr1,Pin1_Motor1,Pin2_Motor1);
    setMotor(dir2,PWM_Motor2,pwr2,Pin1_Motor2,Pin2_Motor2);
    Recoger(e1,e2);
    //printInfo(thetaActual1, thetaActual2, u1, u2, e1, e2);
  }  
}

void plotInfo(float thetaActual1, float thetaActual2){
    Serial.print(theta2);
    Serial.print(" ");
    Serial.print(thetaActual2);
    Serial.print(" ");
    Serial.print(theta1);
    Serial.print(" ");
    Serial.print(thetaActual1);
    Serial.println();
}

void printInfo(float thetaActual1, int thetaActual2, float u1, float u2, float e1, float e2){
    Serial.println("----------------------------------------------------------");
    Serial.println("Motor 1: ");
    Serial.print("theta target: ");
    Serial.println(theta1);
    Serial.print("Theta actual: ");
    Serial.println(thetaActual1);
    Serial.print("Señal de control: ");
    Serial.println(u1);
    Serial.print("Error: ");
    Serial.println(e1);
    Serial.println(" ");
    Serial.println("Motor 2: ");
    Serial.print("theta target: ");
    Serial.println(theta2);
    Serial.print("Theta actual: ");
    Serial.println(thetaActual2);
    Serial.print("Señal de control: ");
    Serial.println(u2);
    Serial.print("Error: ");
    Serial.println(e2);
    Serial.println("**************************");
    Serial.print("Servo: ");
    Serial.println(servoMotor.read());
    Serial.print("Neumatica: ");
    Serial.println(neumatica);
}

void serialEvent(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while(Serial.available() && !newData) {
    rc = (char)Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) {
              ndx = numChars - 1;
          }
      }
      else {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  strcpy(tempChars, receivedChars);
  parseData();
  newData = false;
}

void parseData() {      // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  accion = atoi(strtokIndx); // copy it to messageFromPC

  if(accion == 1){
    Serial.println("got Stop");
    Stop();
  }
  else if (accion == 2){
    Serial.println("got Home");
    Home();
  }
  else if(accion == 3){
    Serial.println("got Yellow");
    delay(1000);
    if(!entrada){
      Take(100,220);
    }
  }
  else if(accion == 4){
    Serial.println("got Red");
    if(!entrada){
      Take(220,0);
    }
  }
  else if(accion == 5){
    Serial.println("got Blue");
    if(!entrada){
      Take(170,170);
    }
  }
  else if(accion == 6){
    Serial.println("got Coordinates");
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    X = atoi(strtokIndx);     // convert this part to an integer
  
    strtokIndx = strtok(NULL, ",");
    Y = atoi(strtokIndx);     // convert this part to a float
    inverseKinematics();
  }
  else if(accion == 7){
    Serial.println("got Angles");
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    theta1 = atoi(strtokIndx);     // convert this part to an integer
    theta1 += 95;
    strtokIndx = strtok(NULL, ",");
    theta2 = atoi(strtokIndx);     // convert this part to a float
    theta2 += 75;
  }
}

void leerEncoder1(){
  //Lectura de Posición 
  int b = digitalRead(Encoder2_Motor1);
  if(b > 0){
    //Incremento variable global
    posi1++;
  }
  else{
    //Decremento variable global
    posi1--;
  }
}

void leerEncoder2(){
  //Lectura de Posición 
  int b = digitalRead(Encoder2_Motor2);
  if(b > 0){
    //Incremento variable global
    posi2++;
  }
  else{
    //Decremento variable global
    posi2--;
  }
}

void setMotor(int dir, int pinPWM, int pwmVal, int pin1, int pin2){
  analogWrite(pinPWM, pwmVal);
  if(dir == 1){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  else if(dir == -1){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else if(dir == 2){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
  }
  else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

void Home(){
  if(contHome > 0){
    contHome =0;
  }
  else{
    contHome++;
    Stop();
    
    setMotor(1,PWM_Motor1,150,Pin1_Motor1,Pin2_Motor1);
    while(digitalRead(Pulsador1_Brazo1) != LOW){
      continue;
    }
    Serial.println("Got limit 1");
    
    setMotor(1,PWM_Motor2,70,Pin1_Motor2,Pin2_Motor2);
    while(digitalRead(Pulsador1_Brazo2) != LOW){
      continue;
    }
    theta1 = 0;
    theta2 = 0;
    thetaRampa1 = 0;
    thetaRampa2 = 0;
    posi1 = 0;
    posi2 = 0;
    
    Serial.println("Got limit 2");
    
    delay(300);
    
    X = 265;
    Y = 0;
    
    inverseKinematics();
    inicio = true;
  }
}

void Stop(){
  setMotor(0,PWM_Motor2,0,Pin1_Motor2,Pin2_Motor2);
  setMotor(0,PWM_Motor1,0,Pin1_Motor1,Pin2_Motor1);
  servoInicial = 0;
  neumaticaFinal = 0;
  inicio = false;
  neumatica = false;
  recogido = false;
  entrada = false;
  digitalWrite(Neumatica,LOW);  
  digitalWrite (servo1, LOW);
  digitalWrite (servo2, LOW);
}

void inverseKinematics(){
  L = getL();
  float gamma = getGamma();
  float alpha = getAlpha();
  float beta = getBeta();
  if(Y < 0){
    theta1 = RAD_to_Grados(alpha + gamma)+95;
    theta2 = RAD_to_Grados(beta- Pi)+75;
  }
  else{
    theta1 = RAD_to_Grados(gamma - alpha)+95;
    theta2 = RAD_to_Grados(Pi - beta)+75;
  }
  checkLimits1();
  checkLimits2();
  Serial.println("----------------------------------------------------------");
  Serial.print("Theta 1: ");
  Serial.println(theta1);
  Serial.print("Theta 2: ");
  Serial.println(theta2);
  Serial.print("X: ");
  Serial.println(X);
  Serial.print("Y: ");
  Serial.println(Y);
  Serial.print("L: ");
  Serial.println(L);
  Serial.print("Gamma: ");
  Serial.println(gamma);
  Serial.print("Alpha: ");
  Serial.println(alpha);
  Serial.print("Beta: ");
  Serial.println(beta);
  Serial.println("----------------------------------------------------------");  
}

float getGamma(){
  return atan2(Y,X);
}

float getAlpha(){
  float a = pow(L,2) + pow(L1,2)- pow(L2,2);
  float b = (2*L1);
  b = b*L;
  a = a/b;
  return acos(min(max(a,-1.0),1.0));
}

float getBeta(){
  float a = -pow(L,2) + pow(L1,2) + pow(L2,2);
  float b = (2*L1);
  b = b*L2;
  a = a/b;
  return acos(min(max(a,-1.0),1.0));
}

float getL(){
  return sqrt(pow(X,2) + pow(Y,2));
}

float RAD_to_Grados(float val){
  float a = val*57.2958;
  return a;
}

float Pulsos_to_Grados1(float pulsos){
  return pulsos * 0.4;
}

float Pulsos_to_Grados2(float pulsos){
  return pulsos * 0.43;
}

int checkLimits1(){
  if(theta1 > 180){
    theta1 = 180; 
  }
  else if(theta1 < 0){
    theta1 = 0;
  }
}

int checkLimits2(){
  if(theta2 > 150){
    theta2 = 150; 
  }
  else if(theta2 < 0){
    theta2 = 0;
  }
}

float PWM1(int control){
  float pwm = fabs(control);
  if( pwm > maxSpeed ){
    pwm = maxSpeed;
  }
  int pwm2 = map(pwm, 0 , maxSpeed, 40, 255);
  return pwm2;
}

float PWM2(int control){
  float pwm = fabs(control);
  if( pwm > maxSpeed ){
    pwm = maxSpeed;
  }
  int pwm2 = map(pwm, 0 , maxSpeed, 30, 255);
  return pwm2;
}

int setDir(int control){
  int dir = 1;
  if(control<0){
    dir = -1;
  }
  return dir;
}
