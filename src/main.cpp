//Robot code to run sensor array


#include <Arduino.h>

double sensorValue1, sensorValue2 = 0.0;
const int irPin0 = 26;  // Analog input pin that the ADC is connected to
const int irPin1 = 25;
const int irPin2 = 33;
const int irPin3 = 32;
const int irPin4 = 35;
const int sensorWidth = 100; //sensor width in miliseconds

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
 
double findWeightedSum (int corrSensorValue[]) {
  
  double linePos = 0.0;
  double totalSum = 0.0;
  
  //Calculates the weighted sum by going over each sensor reading
  for (int pin = 0; pin < 5; pin++) {
      linePos += pin * corrSensorValue[pin];
      totalSum += corrSensorValue[pin];
  }

  linePos /= totalSum;

  //Accoutning for every
  if (totalSum < 0.000001) {
    return 2.0;
  } else {
    return linePos;
  }

  
}