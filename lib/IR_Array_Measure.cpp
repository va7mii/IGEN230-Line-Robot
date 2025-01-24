//Robot code to run sensor array


#include <Arduino.h>

double sensorValue1, sensorValue2 = 0.0;
const int irPin0 = 26;  // Analog input pin that the ADC is connected to
const int irPin1 = 25;
const int irPin2 = 33;
const int irPin3 = 32;
const int irPin4 = 35;
const int sensorWidth = 100; //sensor width in miliseconds


//Initialize motor pins
const int ENA = 13;
const int IN1 = 12;
const int IN2 = 14;
const int IN3 = 27;
const int IN4 = 26;
const int ENB  = 25;

//Settings
const int MAX_MOTOR_SPEED = 400;
const int MAX_MESSAGE_LENGTH = 20;

//Globals
static int leftSpeed = 0;
static int rightSpeed = 0;



//Functions to control motors
void leftMotor (int speed);
void rightMotor (int speed);
void leftBrake(int speed);
void rightBrake(int speed);
void leftStop(void);
void rightStop(void);


int sensorValue[5]; //sensor values raw
int corrSensorValue[5]; //sensor value corrected
int sensorCounter = 0;
double curSensorSum[5]; //doing an array to record multiple values

double findWeightedSum (int corrSensorValue[]);


void setup() {
  // initialize serial communications for ESP-32
  Serial.begin(115200);
  
  //Initialize IR sensor array
  pinMode(irPin0, INPUT);
  pinMode(irPin1, INPUT);
  pinMode(irPin2, INPUT);
  pinMode(irPin3, INPUT);
  pinMode(irPin4, INPUT);
}


void loop() {
  // read the analog in value:
  sensorValue[0] = analogRead(irPin0);
  sensorValue[1] = analogRead(irPin1);
  sensorValue[2] = analogRead(irPin2);
  sensorValue[3] = analogRead(irPin3);
  sensorValue[4] = analogRead(irPin4);

  //Read pot value (test)
  //potValue = analogRead(potInPin);
  
  if (sensorCounter > sensorWidth) {
    
    for (int pin = 0; pin < 5; pin++) {
      corrSensorValue[pin] = curSensorSum[pin]/sensorWidth;
    }
    
    // print the results to the Serial Monitor:
    for (int pin = 0; pin < 5; pin++) {
      Serial.print("sensor");
      Serial.print(pin);
      Serial.print(" = ");
      Serial.print(corrSensorValue[pin]);
      Serial.print(", ");
    }

    double linePos = findWeightedSum(corrSensorValue);
    Serial.print("Position: "); Serial.print(linePos);

    Serial.println(""); //newline

    //reset values
    memset(curSensorSum, 0, sizeof(curSensorSum));
    sensorCounter = 0;
  } else {
    curSensorSum[0] += sensorValue[0];
    curSensorSum[1] += sensorValue[1];
    curSensorSum[2] += sensorValue[2];
    curSensorSum[3] += sensorValue[3];
    curSensorSum[4] += sensorValue[4];
  }
  
  sensorCounter++;
  delay(5);
}
 
double findWeightedSum (int SensorValue[]) {
  
  double linePos = 0.0;
  double totalSum = 0.0;
  
  //Calculates the weighted sum by going over each sensor reading
  for (int pin = 0; pin < 5; pin++) {
      linePos += pin * SensorValue[pin];
      totalSum += SensorValue[pin];
  }

  linePos /= totalSum;

  //Accoutning for every
  if (totalSum < 0.000001) {
    return 2.0;
  } else {
    return linePos;
  }

  
}


//Controls left motor speed; if negative, then switch direction then reverse
void leftMotor (int speed) {
  if (speed >= 0) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(ENA, -speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}


//Controls right motor speed; if negative, then switch direction to reverse
void rightMotor (int speed) {
  if (speed >= 0) {
    analogWrite(ENB, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENB, -speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

//Turns both IN1 and IN2 pins HIGH to brake left motor
void leftBrake(int speed) {
  analogWrite(ENA, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

//Turns both IN3 and IN4 pins to HIGH to brake right motor
void rightBrake(int speed) {
  analogWrite(ENB, speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
}


//Turns both IN1 and IN2 pins LOW to stop left motor
void leftStop(void) {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

//Turns both IN3 and IN4 pins to LOW to stop right motor
void rightStop(void) {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
}