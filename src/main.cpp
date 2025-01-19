//References
//https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-2-freertos/b3f84c9c9455439ca2dcb8ccfce9dec5
//Example Sketches for BluetoothSerial and FreeRTOS on ESP32


#include <Arduino.h>

#include <stdlib.h>
#include "BluetoothSerial.h"

//Constrict ESP-32 to only one core 0 since haven't done multicore stuff yet
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

/*
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;
*/

//Initialize motor pins
const int ENA = 19;
const int IN1 = 18;
const int IN2 = 5;
const int IN3 = 17;
const int IN4 = 16;
const int ENB  = 4;


//Iniitialize IR sensor pins
double sensorValues[5];
double whiteSensorValues[5];
double blackSensorValues[5];
const int irPins[] = {26,25,33,32,35};  // Analog input pin that the ADC is connected to
const int sensorWidth = 100; //sensor width in miliseconds

//Some things to check calibration



//Configure Bluetooth
String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

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

//Read serial monitor and change motor speed based on user input
//[l or r] [motor speed]
//e.g. r 50, change right motor to 50 speed
void readSerial(void *parameter) {
  while(1) {
    while (SerialBT.available() > 0) {
      //Place to hold incoming messages
      static char message[MAX_MESSAGE_LENGTH];
      static unsigned int message_pos = 0;

      
      char inByte = SerialBT.read();
      char whichCommand = NULL; //command to be called
      int number;

      //Is enter typed or message exceeds max size
      if (inByte == '\n' || message_pos > MAX_MESSAGE_LENGTH) {
        //Convert message into an integer
        char* tail = NULL; //get the number after the letter command
        int number = 0;

        //Check which command was called
        switch (message[0]) {
          case 'l':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            //Adjust speed
            leftSpeed = number;
            SerialBT.print("Updated left motor speed to: " );
            SerialBT.println(leftSpeed);
            break;
          case 'r':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            rightSpeed = number;
            SerialBT.print("Updated right motor speed to: " );
            SerialBT.println(rightSpeed);
            break;

          case 'c':
            //Calibrate IR sensor

          case 'm':
            //Switch to remote control using a Switch Pro controller

          case 'a':
            //Revert back to line-following mode
          default:
            SerialBT.println("Invalid command!");
        }


        //Resets message and message_pos
        message_pos = 0;
        memset(message, 0, MAX_MESSAGE_LENGTH);
        
      } else {
        //Store incoming character into message
        message[message_pos] = inByte;
        message_pos++;

        //Check which command is called
 
      }
    }
  }

}

//Task for PID line following
void followLine(void *parameter){
  while (1) {
    leftMotor(leftSpeed);
    rightMotor(rightSpeed);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    leftStop();
    rightStop();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void readIR(void *parameter) {
  while (1) {
    // read the analog in value:
    for (int i = 0; i < 5; i++) {
      sensorValues[i] = analogRead(irPins[i]);
    }

    // print the results to the Serial Monitor:
    for (int pin = 0; pin < 5; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");
    }

    SerialBT.println();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}




void setup() {


  //Initialize motor control pins
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);

  //Initialize each IR pins
  for (int i = 0; i < 5; i++) {
    pinMode(irPins[i], INPUT);
  }



  // Start serial monitor
  Serial.begin(115200);
  //Activate bluetooth
  SerialBT.begin(device_name); 

  vTaskDelay(1000 / portTICK_PERIOD_MS); //wait for a second to let serial output work
  Serial.println("Bluetooth ready! Please connect a serial terminal from another device to the ESP32");
  SerialBT.println("Enter a speed");


  //Initial calibration

  //Create tasks
  xTaskCreatePinnedToCore(readSerial, "readSerial", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(followLine, "followLine", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(readIR, "Read IR", 1024, NULL, 1, NULL, app_cpu);

  //Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  //Don't add anything here for RTOS
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

void calibrateIR(void) {

  SerialBT.println("//////////////Initial calibration started!////// ");
  Serial.println("///////////////Initial calibration started!/////");



  //White track calibration

  while (1)
  {
    // read the analog in value:
    for (int i = 0; i < 5; i++) {
      sensorValues[i] = analogRead(irPins[i]);
    }

    // print the results to the Serial Monitor:
    for (int pin = 0; pin < 5; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");
    }

    SerialBT.println();

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }


  //Black line calibration


}
