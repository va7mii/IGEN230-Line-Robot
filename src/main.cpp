//References
//https://www.digikey.com/en/maker/projects/introduction-to-rtos-solution-to-part-2-freertos/b3f84c9c9455439ca2dcb8ccfce9dec5
//Example Sketches for BluetoothSerial and FreeRTOS on ESP32
//PID source: https://www.teachmemicro.com/implementing-pid-for-a-line-follower-robot/


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
double corrSensorValues[5]; //calibrated sensor values

const int irPins[] = {26, 25, 35, 34,A0}; // Analog input pin that the ADC is connected to

const int sensorWidth = 100; //sensor width in miliseconds

//25, 33, 32, 35, 34,
//16,17,5,18, (4,15) pwm





int numPins = sizeof(irPins) / sizeof(int);
double linePos = 2.0;

//PID parameters

float Kp = 40;
float Ki = 0;
float Kd = 40;

// Shared variables
volatile float error = 0.0;         // Line position error
volatile float controlOutput = 0.0; // PID output for motor adjustment
volatile bool newSensorData = false; // Flag to indicate new sensor data


//Some things to check calibration
volatile int calibratedWhite = LOW;
volatile int calibratedBlack = LOW;

TaskHandle_t autoHandle = NULL;
TaskHandle_t manualHandle = NULL;

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

int leftSpeed = 0;
int rightSpeed = 0;

//Globals
static int manualleftSpeed = 0;
static int manualrightSpeed = 0;



//Functions to control motors
void leftMotor (int speed);
void rightMotor (int speed);
void leftBrake(int speed);
void rightBrake(int speed);
void leftStop(void);
void rightStop(void);

//Functions to calibrate IR
void calibrateIR(void);

//Function to calculate IR sensor values during
void readIR(void);

//Function to find weighted IR Sum
double findWeightedSum(double sensorValue[]);

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
            manualleftSpeed = number;
            SerialBT.print("Updated left motor speed to: " );
            SerialBT.println(manualleftSpeed);
            break;
          case 'r':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            manualrightSpeed = number;
            SerialBT.print("Updated right motor speed to: " );
            SerialBT.println(manualrightSpeed);
            break;

          case 'p':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            //Adjust Kp
            Kp = number;
            SerialBT.print("Updated Kp to: " );
            SerialBT.println(Kp);
            break;

          case 'i':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            //Adjust Kp
            Ki = number;
            SerialBT.print("Updated Ki to: " );
            SerialBT.println(Ki);
            break;
          
          case 'd':
            //Extracts number from input
            tail = message + 1;
            number = atoi(tail);

            //Adjust Kp
            Kd = number;
            SerialBT.print("Updated Kd to: " );
            SerialBT.println(Kd);
            break;

          case 'c':
            if (!calibratedWhite) {
              calibratedWhite = HIGH;
            } else { //White base already calibrated
              calibratedBlack = HIGH;
            }
            break;

          case 'm':
            SerialBT.println("Manual mode toggled!");
            vTaskSuspend(autoHandle);
            vTaskResume(manualHandle);

            break;

          case 'a':
            vTaskSuspend(manualHandle);
            vTaskResume(autoHandle);

            break;
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

/*
void slidingIRWindow (){
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
}
*/


//Task for PID line following
void followLine(void *parameter){
  while (1) {

      // PID variables
    float integral = 0.0;
    float prevError = 0.0;

    while (true) {
      // Wait for new sensor data
      if (newSensorData) {
        // Reset the new data flag
        newSensorData = false;

        // Calculate PID components
        integral += error;                         // Accumulate error (integral term)
        float derivative = error - prevError;      // Calculate rate of change (derivative term)
        controlOutput = (Kp * error) + (Ki * integral) + (Kd * derivative); // PID output

        // Save error for next iteration
        prevError = error;
      }



      // Allow other tasks to run
      vTaskDelay(pdMS_TO_TICKS(5)); // Adjust as needed for PID computation rate
    }
  }
}


void sensorTask(void *parameter) {
  while (true) {


    for (int pin = 0; pin < numPins; pin++) {
      sensorValues[pin] = analogRead(irPins[pin]);
      corrSensorValues[pin] = map(sensorValues[pin], whiteSensorValues[pin], blackSensorValues[pin], 0, 255);

      //Adjusting overshoot values
      if (corrSensorValues[pin] < 0) {
        corrSensorValues[pin] = 0;
      }

      if (corrSensorValues[pin] > 255) {
        corrSensorValues[pin] = 0;
      }
    }

    // print the results to the Serial Monitor:
    /* for (int pin = 0; pin < numPins; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(corrSensorValues[pin]);
      SerialBT.print(", ");
    } */

    // Prints weighted sum position
    linePos = findWeightedSum(corrSensorValues);

    // Update shared error value
    error = 2.0 - linePos;  // Negative to align error direction with desired correction

    // Set flag to indicate new sensor data
    newSensorData = true;

    Serial.println(linePos);

    // Delay for sensor reading rate (e.g., 10 ms)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// Task: Motor Control Task
void motorTask(void *parameter) {
  const int baseSpeed = 75; // Base motor speed (PWM value)

  while (true) {
    // Calculate motor speeds based on PID control output
    int leftMotorSpeed = baseSpeed + controlOutput;  // Reduce left motor speed
    int rightMotorSpeed = baseSpeed - controlOutput; // Increase right motor speed

    // Constrain speeds to valid PWM range (0-255)
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    leftMotor(leftMotorSpeed);
    rightMotor(rightMotorSpeed);


    // Allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(10)); // Adjust motor update rate
  }
}

// Remote control mode
void manualControl(void *parameter){
  while (1) {
    leftMotor(manualleftSpeed);
    rightMotor(manualrightSpeed);
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
  for (int i = 0; i < numPins; i++) {
    pinMode(irPins[i], INPUT);
  }



  // Start serial monitor
  Serial.begin(115200);
  //Activate bluetooth
  SerialBT.begin(device_name); 



  vTaskDelay(1000 / portTICK_PERIOD_MS); //wait for a second to let serial output work
  Serial.println("Bluetooth ready! Please connect a serial terminal from another device to the ESP32");
  SerialBT.println("Enter a speed");

  //Start bluetooth serial task
  xTaskCreatePinnedToCore(readSerial, "readSerial", 1024, NULL, 1, NULL, app_cpu);

  //Calibrate sensors
  calibrateIR(); 

  //Create line follow tasks
  xTaskCreatePinnedToCore(followLine, "followLine", 1024, NULL, 1, &autoHandle, app_cpu);
  xTaskCreatePinnedToCore(motorTask, "motorTask", 1024, NULL, 1, &autoHandle, app_cpu);
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 1024, NULL, 1, &autoHandle, app_cpu);
  xTaskCreatePinnedToCore(manualControl, "manualControl", 1024, NULL, 1, &manualHandle, app_cpu);


  //Initially suspend the manual task
  vTaskSuspend(manualHandle);

  //Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  //Don't add anything here for RTOS
}




//// HELPER FUNCTIONS ///////
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

  while (!calibratedWhite) {
    // read the analog in value:
    for (int i = 0; i < numPins; i++) {
      sensorValues[i] = analogRead(irPins[i]);
    }

    SerialBT.print("Calibrating white base: ");
    // print the results to the Serial Monitor:
    for (int pin = 0; pin < numPins; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");
    }

    SerialBT.println();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  

  // Stores calibrated white base values
  SerialBT.print("White base calibrated as: ");

  for (int pin = 0; pin < numPins; pin++) {
      whiteSensorValues[pin] = sensorValues[pin];

      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);



// Calibrate black line values (now take the max value)

  int prevValues[5] = {0};

  while (!calibratedBlack)
  {
    // read the analog in value:
    for (int i = 0; i < numPins; i++) {
      sensorValues[i] = analogRead(irPins[i]);

      //Don't touch if new value is lesser
      if (sensorValues[i] < prevValues[i]) {
        sensorValues[i] = prevValues[i];
      }
    }

    // print the results to the Serial Monitor:
    SerialBT.print("Calibrating black line: ");
    for (int pin = 0; pin < numPins; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");

      prevValues[pin] = sensorValues[pin];
    }

    SerialBT.println();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  // Stores calibrated white base values
  SerialBT.print("Black line calibrated as: ");

  for (int pin = 0; pin < numPins; pin++) {
      blackSensorValues[pin] = sensorValues[pin];
        
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(sensorValues[pin]);
      SerialBT.print(", ");
  
  }

  vTaskDelay(500 / portTICK_PERIOD_MS);


}


double findWeightedSum (double sensorValues[]) {
  
  double linePos = 0.0;
  double totalSum = 0.0;
  
  //Calculates the weighted sum by going over each sensor reading
  for (int pin = 0; pin < numPins; pin++) {
      linePos += pin * sensorValues[pin];
      totalSum += sensorValues[pin];
  }

  linePos /= totalSum;

  //Accoutning for every
  if (totalSum < 0.000001) {
    return 2.0;
  } else {
    return linePos;
  }

}


void readIR () {
  // read the analog in value:
    for (int pin = 0; pin < numPins; pin++) {
      sensorValues[pin] = analogRead(irPins[pin]);
      corrSensorValues[pin] = map(sensorValues[pin], whiteSensorValues[pin], blackSensorValues[pin], 0, 255);

      //Adjusting overshoot values
      if (corrSensorValues[pin] < 0) {
        corrSensorValues[pin] = 0;
      }

      if (corrSensorValues[pin] > 255) {
        corrSensorValues[pin] = 0;
      }
    }

    // print the results to the Serial Monitor:
    /* for (int pin = 0; pin < numPins; pin++) {
      SerialBT.print("sensor");
      SerialBT.print(pin);
      SerialBT.print(" = ");
      SerialBT.print(corrSensorValues[pin]);
      SerialBT.print(", ");
    } */

    // Prints weighted sum position
    linePos = findWeightedSum(corrSensorValues);
    SerialBT.print("Position: "); SerialBT.print(linePos);

}