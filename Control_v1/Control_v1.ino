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
#define PWM_Motor1  4
#define PWM_Motor2  5

//Definicion de los pines que leen los encoders de los motores, tiene que ser de interrupciones
#define Encoder1_Motor1  2
#define Encoder2_Motor1  6
#define Encoder1_Motor2  3
#define Encoder2_Motor2  7

//Definicion del pin del servomotor
 #define servo  8

//Deficinion del pin del motor de emergencia, tiene que ser de interrupciones
#define Encendido  19

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
  //pinMode(Pulsador2_Brazo1, INPUT_PULLUP);
  pinMode(Pulsador1_Brazo2, INPUT_PULLUP);
  //pinMode(Pulsador2_Brazo2, INPUT_PULLUP);
  //Servo motor
  servoMotor.attach(servo);
  //Neumatica
  pinMode(Neumatica, OUTPUT);
  //PWM
  pinMode(PWM_Motor1, OUTPUT);
  pinMode(PWM_Motor2, OUTPUT);

  Serial.println("target pos");
}

void recoger(){
  digitalWrite(Neumatica, HIGH);
  servoMotor.write(90);
  delay(5);
  servoMotor.write(0);
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
    
    setMotor(1,PWM_Motor1,90,Pin1_Motor1,Pin2_Motor1);
    while(digitalRead(Pulsador1_Brazo1) != LOW){
      continue;
    }
    Serial.println("Got limit 1");
    
    setMotor(1,PWM_Motor2,90,Pin1_Motor2,Pin2_Motor2);
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
  inicio = false;
}

void inverseKinematics(){
  L = getL();
  float gamma = getGamma();
  float alpha = getAlpha();
  float beta = getBeta();

  theta1 = RAD_to_Grados(gamma - alpha)+90;
  theta2 = RAD_to_Grados(beta - Pi)+80;
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

float Pulsos_to_Grados(float pulsos){
  return pulsos * 15/13;
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
  if(theta2 > 160){
    theta2 = 160; 
  }
  else if(theta2 < 0){
    theta2 = 0;
  }
}

float PWM(int control){
  float pwm = fabs(control);
  if( pwm > maxSpeed ){
    pwm = maxSpeed;
  }
  pwm2 = map(pwm, 0 , maxSpeed, 30, 255);
  return pwm2;
}

int setDir(int control){
  int dir = 1;
  if(control<0){
    dir = -1;
  }
  return dir;
}

void loop(){   

  if(!inicio){
    Stop();
  }
  else{
    if(thetaRampa1 < theta1){
      thetaRampa1++;
    }
    else if(thetaRampa1 > theta1){
      thetaRampa1--;
    }
    if(thetaRampa2 < theta2){
      thetaRampa2++;
    }
    else if(thetaRampa2 > theta2){
      thetaRampa2--;
    }
    
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    
    float thetaActual1 = 0;
    float thetaActual2 = 0; 
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      thetaActual1 = Pulsos_to_Grados(posi1);
      thetaActual2 = Pulsos_to_Grados(posi2);
    }
    // error
    float e1 = thetaActual1 - thetaRampa1;
    float e2 = thetaActual2 - thetaRampa2;
    
    // derivative
    float dedt1 = (e1-eprev1)/(deltaT);
    float dedt2 = (e2-eprev2)/(deltaT);

    // control signal
    float u1 = kp1*e1 + Kd*dedt1;
    float u2 = kp2*e2 + Kd*dedt2;
    
    eprev1 = e1;
    eprev2 = e2;
    
    float pwr1 = PWM(u1);
    float pwr2 = PWM(u2);
    int dir1 = setDir(u1);
    int dir2 = setDir(u2);

    setMotor(dir1,PWM_Motor1,pwr1,Pin1_Motor1,Pin2_Motor1);
    setMotor(dir2,PWM_Motor2,pwr2,Pin1_Motor2,Pin2_Motor2);

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
  }  
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
    Serial.println("got Coordinates");
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    X = atoi(strtokIndx);     // convert this part to an integer
  
    strtokIndx = strtok(NULL, ",");
    Y = atoi(strtokIndx);     // convert this part to a float
    inverseKinematics();
  }
  else if(accion == 4){
    Serial.println("got Angles");
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    theta1 = atoi(strtokIndx);     // convert this part to an integer
    theta1 += 90;
    strtokIndx = strtok(NULL, ",");
    theta2 = atoi(strtokIndx);     // convert this part to a float
    theta2 += 80;
  }
  else if(accion == 5){
    Serial.println("Got pick up");
    recoger();
  }
  else if(accion == 6){
    Serial.println("got let go");
    digitalWrite(Neumatica,LOW);
  }
}
